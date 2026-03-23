#include "configs.h"

#ifdef Use_CAS

#include "trs80cas.h"
#include "Arduino.h"
#include "MaxDuino.h"
#include "processing_state.h"
#include "MaxProcessing.h"
#include "file_utils.h"
#include "compat.h"

namespace {

enum class TRS80Mode : uint8_t {
  NONE = 0,
  BASIC_L1,
  BASIC_L2,
  SYSTEM_L1,
  SYSTEM_L2,
  HIGHSPEED,
};

static TRS80Mode trs80_mode = TRS80Mode::NONE;
static unsigned long trs80_payload_start = 0;
static bool trs80_sent_direct_header = false;
static uint32_t trs80_leading_silence = 0;
static uint32_t trs80_trailing_silence = 0;
static byte trs80_current_byte = 0;
static uint8_t trs80_bit_mask = 0;
static uint8_t trs80_current_bit_value = 0;
static uint8_t trs80_low_half_phase = 0;
static uint16_t trs80_low_half_pos = 0;
static uint16_t trs80_low_half_samples = 0;
static uint8_t trs80_high_phase = 0;
static uint16_t trs80_high_remaining = 0;
static double trs80_frac_error = 0.0;
static bool trs80_eof = false;

static constexpr uint16_t TRS80_SAMPLE_US = 45; // ~= 22.05 kHz direct sample clock
static constexpr uint32_t TRS80_SILENCE_SAMPLES = 11025; // 0.5s at 22050 Hz
static constexpr double TRS80_CLOCK_HZ = 2027520.0;
static constexpr byte TRS80_MARKER_PROBE_LEN = 20;
static constexpr unsigned long TRS80_RAW_SCAN_LIMIT = 512;

static bool is_header_delim(char c) {
  return c == '\0' || c == '\r' || c == '\n' || c == ' ' || c == '\t' || c == ':' || c == ';' || c == ',' || c == ']';
}

static bool parse_marker(const char *s, TRS80Mode &mode, size_t &marker_len) {
  struct Marker { const char *name; TRS80Mode mode; };
  static const Marker markers[] = {
    {"basic-l1", TRS80Mode::BASIC_L1},
    {"basic-l2", TRS80Mode::BASIC_L2},
    {"system-l1", TRS80Mode::SYSTEM_L1},
    {"system-l2", TRS80Mode::SYSTEM_L2},
    {"highspeed", TRS80Mode::HIGHSPEED},
  };

  size_t start = 0;
  if ((uint8_t)s[0] == 0xEF && (uint8_t)s[1] == 0xBB && (uint8_t)s[2] == 0xBF) {
    start = 3;
  }
  if (s[start] == '[') {
    start += 1;
  }

  for (const auto &m : markers) {
    size_t n = strlen(m.name);
    if (strncasecmp(s + start, m.name, n) == 0 && is_header_delim(s[start + n])) {
      mode = m.mode;
      marker_len = start + n;
      while (is_header_delim(s[marker_len])) {
        marker_len += 1;
      }
      return true;
    }
  }

  return false;
}

static bool read_probe_byte(unsigned long offset, byte &value) {
  if (offset >= filesize) {
    return false;
  }

  if (readfile(1, offset) != 1) {
    return false;
  }

  value = filebuffer[0];
  return true;
}

static bool find_raw_sync(unsigned long &sync_offset) {
  const unsigned long limit = (filesize < TRS80_RAW_SCAN_LIMIT) ? filesize : TRS80_RAW_SCAN_LIMIT;
  byte value = 0;
  unsigned long offset = 0;

  while (offset < limit) {
    if (!read_probe_byte(offset, value)) {
      return false;
    }

    if (value != 0) {
      break;
    }

    offset += 1;
  }

  if (offset >= limit || value != 0xA5) {
    return false;
  }

  sync_offset = offset;
  return true;
}

static bool is_printable_ascii(byte value) {
  return value >= 0x20 && value <= 0x7E;
}

static bool detect_raw_level2(TRS80Mode &mode) {
  unsigned long sync_offset = 0;
  byte value = 0;

  if (!find_raw_sync(sync_offset) || !read_probe_byte(sync_offset + 1, value)) {
    return false;
  }

  if (value == 0x55) {
    byte block_header = 0;
    if (!read_probe_byte(sync_offset + 8, block_header)) {
      return false;
    }

    for (byte i = 0; i < 6; ++i) {
      byte name_byte = 0;
      if (!read_probe_byte(sync_offset + 2 + i, name_byte) || !is_printable_ascii(name_byte)) {
        return false;
      }
    }

    if (block_header == 0x3C) {
      byte block_len = 0;
      if (!read_probe_byte(sync_offset + 9, block_len)) {
        return false;
      }

      const unsigned long data_len = (block_len == 0) ? 256UL : (unsigned long)block_len;
      const unsigned long checksum_offset = sync_offset + 12 + data_len;
      if (checksum_offset >= filesize) {
        return false;
      }
    } else if (block_header != 0x78) {
      return false;
    }

    mode = TRS80Mode::SYSTEM_L2;
    return true;
  }

  byte next = 0;
  byte next2 = 0;
  if (!read_probe_byte(sync_offset + 2, next) || !read_probe_byte(sync_offset + 3, next2)) {
    return false;
  }

  if (value == 0xD3 && next == 0xD3 && next2 == 0xD3) {
    mode = TRS80Mode::BASIC_L2;
    return true;
  }

  return false;
}

static bool detect_raw_level1(TRS80Mode &mode) {
  unsigned long sync_offset = 0;
  byte start_hi = 0;
  byte start_lo = 0;
  byte end_hi = 0;
  byte end_lo = 0;

  if (!find_raw_sync(sync_offset) ||
      !read_probe_byte(sync_offset + 1, start_hi) ||
      !read_probe_byte(sync_offset + 2, start_lo) ||
      !read_probe_byte(sync_offset + 3, end_hi) ||
      !read_probe_byte(sync_offset + 4, end_lo)) {
    return false;
  }

  const uint16_t start = word(start_hi, start_lo);
  const uint16_t end = word(end_hi, end_lo);
  if (end < start) {
    return false;
  }

  const unsigned long data_len = (unsigned long)(end - start) + 1UL;
  const unsigned long checksum_offset = sync_offset + 5UL + data_len;
  if (checksum_offset >= filesize) {
    return false;
  }

  if (start == 0x4200) {
    mode = TRS80Mode::BASIC_L1;
    return true;
  }

  if (start >= 0x4000 && start < 0x4200) {
    mode = TRS80Mode::SYSTEM_L1;
    return true;
  }

  return false;
}

static bool detect_raw_mode(TRS80Mode &mode, unsigned long &payload_start) {
  if (detect_raw_level2(mode) || detect_raw_level1(mode)) {
    payload_start = 0;
    return true;
  }

  return false;
}

static bool init_mode(TRS80Mode mode, unsigned long payload_start) {
  trs80_mode = mode;
  trs80_payload_start = payload_start;
  trs80_sent_direct_header = false;
  trs80_leading_silence = TRS80_SILENCE_SAMPLES;
  trs80_trailing_silence = TRS80_SILENCE_SAMPLES;
  trs80_frac_error = 0.0;
  trs80_eof = false;
  bytesRead = trs80_payload_start;
  reset_byte_state();

  switch (trs80_mode) {
    case TRS80Mode::BASIC_L1:
    case TRS80Mode::SYSTEM_L1:
      trs80_low_half_samples = 44; // 88 samples/bit => 250 baud
      break;

    case TRS80Mode::BASIC_L2:
    case TRS80Mode::SYSTEM_L2:
      trs80_low_half_samples = 22; // 44 samples/bit => 500 baud
      break;

    case TRS80Mode::HIGHSPEED:
      trs80_low_half_samples = 0;
      break;

    default:
      return false;
  }

  currentID = BLOCKID::TRS80CAS;
  currentTask = TASK::PROCESSID;
  return true;
}

static int duration_to_samples(double seconds) {
  double exact = seconds * 22050.0 + trs80_frac_error;
  int whole = (int) exact;
  trs80_frac_error = exact - whole;
  if (whole < 1) {
    whole = 1;
  }
  return whole;
}

static int tcycles_to_samples(int tcycles) {
  return duration_to_samples((double)tcycles / TRS80_CLOCK_HZ);
}

static void reset_byte_state() {
  trs80_current_byte = 0;
  trs80_bit_mask = 0;
  trs80_current_bit_value = 0;
  trs80_low_half_phase = 0;
  trs80_low_half_pos = 0;
  trs80_high_phase = 0;
  trs80_high_remaining = 0;
}

static bool trs80_load_next_byte() {
  if (!ReadByte()) {
    trs80_eof = true;
    reset_byte_state();
    return false;
  }
  trs80_current_byte = outByte;
  trs80_bit_mask = 0x80;
  trs80_current_bit_value = 0;
  trs80_low_half_phase = 0;
  trs80_low_half_pos = 0;
  trs80_high_phase = 0;
  trs80_high_remaining = 0;
  return true;
}

static bool trs80_next_low_speed_level(uint8_t *level) {
  if (trs80_bit_mask == 0) {
    if (!trs80_load_next_byte()) {
      return false;
    }
  }

  if (trs80_low_half_pos == 0) {
    trs80_current_bit_value = (trs80_current_byte & trs80_bit_mask) ? 1 : 0;
  }

  if (trs80_low_half_phase == 0) {
    *level = (trs80_low_half_pos < 3) ? 1 : 0;
  } else {
    *level = (trs80_current_bit_value && trs80_low_half_pos < 3) ? 1 : 0;
  }

  trs80_low_half_pos += 1;
  if (trs80_low_half_pos >= trs80_low_half_samples) {
    trs80_low_half_pos = 0;
    if (trs80_low_half_phase == 0) {
      trs80_low_half_phase = 1;
    } else {
      trs80_low_half_phase = 0;
      trs80_bit_mask >>= 1;
    }
  }

  return true;
}

static bool trs80_next_high_speed_level(uint8_t *level) {
  if (trs80_bit_mask == 0) {
    if (!trs80_load_next_byte()) {
      return false;
    }
  }

  if (trs80_high_remaining == 0) {
    trs80_current_bit_value = (trs80_current_byte & trs80_bit_mask) ? 1 : 0;
    if (trs80_high_phase == 0) {
      trs80_high_remaining = trs80_current_bit_value ? tcycles_to_samples(378) : tcycles_to_samples(771);
    } else {
      trs80_high_remaining = trs80_current_bit_value ? tcycles_to_samples(381) : tcycles_to_samples(765);
    }
  }

  *level = (trs80_high_phase == 0) ? 1 : 0;

  trs80_high_remaining -= 1;
  if (trs80_high_remaining == 0) {
    if (trs80_high_phase == 0) {
      trs80_high_phase = 1;
    } else {
      trs80_high_phase = 0;
      trs80_bit_mask >>= 1;
    }
  }

  return true;
}

static bool trs80_next_level(uint8_t *level) {
  if (trs80_leading_silence) {
    trs80_leading_silence -= 1;
    *level = 0;
    return true;
  }

  if (!trs80_eof) {
    switch (trs80_mode) {
      case TRS80Mode::BASIC_L1:
      case TRS80Mode::SYSTEM_L1:
      case TRS80Mode::BASIC_L2:
      case TRS80Mode::SYSTEM_L2:
        if (trs80_next_low_speed_level(level)) {
          return true;
        }
        break;

      case TRS80Mode::HIGHSPEED:
        if (trs80_next_high_speed_level(level)) {
          return true;
        }
        break;

      default:
        break;
    }
  }

  if (trs80_trailing_silence) {
    trs80_trailing_silence -= 1;
    *level = 0;
    return true;
  }

  return false;
}

} // namespace

bool trs80cas_detect_and_init() {
  const byte header_len = (filesize < TRS80_MARKER_PROBE_LEN) ? (byte)filesize : TRS80_MARKER_PROBE_LEN;
  if (header_len == 0 || readfile(header_len, 0) != header_len) {
    return false;
  }

  char hdr[TRS80_MARKER_PROBE_LEN + 1];
  memset(hdr, 0, sizeof(hdr));
  memcpy(hdr, filebuffer, header_len);

  TRS80Mode mode = TRS80Mode::NONE;
  size_t marker_len = 0;
  unsigned long payload_start = 0;

  if (parse_marker(hdr, mode, marker_len)) {
    payload_start = marker_len;
  } else if (!detect_raw_mode(mode, payload_start)) {
    return false;
  }

  return init_mode(mode, payload_start);
}

void trs80cas_process() {
  if (!trs80_sent_direct_header) {
    trs80_sent_direct_header = true;
    currentPeriod = word(0x6000 | TRS80_SAMPLE_US);
    return;
  }

  uint8_t bits = 0;
  uint8_t count = 0;
  while (count < 8) {
    uint8_t level = 0;
    if (!trs80_next_level(&level)) {
      break;
    }
    bits = (bits << 1) | (level ? 1 : 0);
    count += 1;
  }

  if (count == 0) {
    currentID = BLOCKID::IDEOF;
    currentTask = TASK::PROCESSID;
    count_r = 255;
    currentPeriod = 0;
    return;
  }

  bits <<= (8 - count);
  currentPeriod = word(0x4000 | ((count - 1) << 8) | bits);
}

#endif // Use_CAS
