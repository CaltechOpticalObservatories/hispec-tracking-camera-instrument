/**
 * @file    fits_header_dictionary.cpp
 * @brief   FITS keyword dictionary — property -> keyword/comment/default/enum
 */

#include "fits_header_dictionary.h"

namespace Camera {

  namespace {
    using Type = HeaderValueType;
  }

  const std::vector<HeaderDictEntry>& fits_header_dictionary() {
    // bitpix->DETBITS: BITPIX is a reserved CFITSIO keyword.
    // pixel_time->PIXTIME: PIXELTIME is 9 chars, over the FITS keyword limit.
    static const std::vector<HeaderDictEntry> dictionary = {
      { "mjd_start", "EXPMJDST",
        "Exposure start Modified Julian Date of first read",
        Type::Number, "", "", {} },
      { "read_mode", "READMODE", "Readout mode",
        Type::String, "", "", {} },
      { "subframe_mode", "SUBFRAME",
        "Subframe mode, e.g. guiding, ROI, or fullframe",
        Type::String, "", "", {} },
      { "operational_mode", "OPSMODE", "Operational mode, e.g. freerun or autofetch",
        Type::String, "", "", {} },
      { "exposure_time", "EXPTIME", "Effective exposure time",
        Type::Number, "", "0", {} },
      { "bitpix", "DETBITS", "Detector bit depth, needed for offset correction",
        Type::Integer, "16", "16", {} },
      { "clock_rate", "CLOCKRAT", "Clock rate of the detector readout",
        Type::Number, "", "", {} },
      { "pixel_time", "PIXTIME", "Pixel time spent reading out each pixel",
        Type::Number, "5.92", "", {} },
      { "frame_time", "FRAMETME", "Time to read out each amplifier region",
        Type::Number, "", "", {} },
      { "n_channels", "NCHANLS", "Number of detector channels",
        Type::Integer, "4", "64", {} },
      { "refpix_channel", "REFPXAMP", "The channel corresponding to the reference pixel",
        Type::Integer, "5", "", {} },
      { "ref_channel_position", "REFCHPOS",
        "Position of the reference channel in the data array",
        Type::String, "right", "RIGHT", {"TOP", "BOTTOM", "LEFT", "RIGHT", "NONE"} },
      { "channels_are_vertical", "CHANVERT",
        "Are the spectral channels aligned vertically in the data array?",
        Type::Boolean, "N/A", "TRUE", {} },
      { "filename", "FILENAME", "Name of the file",
        Type::String, "", "", {} },
      { "file_type", "FILETYPE", "Type of file",
        Type::String, "", "FITS", {"FITS"} },
      { "LVLC_V1", "LVLC_V1", "Gate voltage",
        Type::Number, "from ACF", "", {} },
      { "LVLC_V2", "LVLC_V2", "Bias power",
        Type::Number, "from ACF", "", {} },
      { "LVLC_V3", "LVLC_V3", "Diode substrate voltage",
        Type::Number, "from ACF", "", {} },
      { "CAMD_VER", "CAMD_VER", "camerad build date",
        Type::String, "", "", {} },
      { "FIRMWARE", "FIRMWARE", "Controller ACF file used",
        Type::String, "", "", {} },
      { "nskip_rows", "SKIPROWS", "Number of rows skipped",
        Type::Integer, "", "", {} },
      { "nskip_lines", "SKIPLNES", "Number of lines skipped",
        Type::Integer, "", "", {} },
      { "n_reads", "NREADS", "Number of reads in this exposure (freerun: running frame counter)",
        Type::Integer, "", "", {} },
      { "acq_time", "ACQTIME", "UTC time of acquisition",
        Type::String, "", "", {} },
      { "FREERUN", "FREERUN", "Exposure was acquired in freerun mode",
        Type::Boolean, "FALSE", "FALSE", {} },
    };
    return dictionary;
  }

  const HeaderDictEntry* find_header_entry(const std::string& property) {
    for (const auto& entry : fits_header_dictionary()) {
      if (entry.property == property) return &entry;
    }
    return nullptr;
  }

  const std::string& header_default(const HeaderDictEntry& entry, FitsCamera camera) {
    return (camera == FitsCamera::ATC) ? entry.default_atc : entry.default_spec;
  }

  bool is_valid_header_enum(const HeaderDictEntry& entry, const std::string& value) {
    if (entry.enum_values.empty()) return true;
    for (const auto& allowed : entry.enum_values) {
      if (allowed == value) return true;
    }
    return false;
  }

}
