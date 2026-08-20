/**
 * @file    fits_header_dictionary.h
 * @brief   FITS keyword dictionary — property -> keyword/comment/default/enum
 */
#pragma once

#include <string>
#include <vector>

namespace Camera {

  enum class HeaderValueType { Number, Integer, String, Boolean };

  enum class FitsCamera { ATC, SPEC };

  struct HeaderDictEntry {
    std::string property;
    std::string keyword;
    std::string comment;
    HeaderValueType type;
    std::string default_atc;              // empty = no default
    std::string default_spec;             // empty = no default
    std::vector<std::string> enum_values; // empty = unconstrained
  };

  const std::vector<HeaderDictEntry>& fits_header_dictionary();

  const HeaderDictEntry* find_header_entry(const std::string& property);

  const std::string& header_default(const HeaderDictEntry& entry, FitsCamera camera);

  bool is_valid_header_enum(const HeaderDictEntry& entry, const std::string& value);

}
