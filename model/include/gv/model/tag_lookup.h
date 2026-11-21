// tag36h11 ID lookup golden model (PLAN step 6, "ID lookup, error correction"),
// following NAPA's nearest_code() and errors_connected() (zwatx/NAPA,
// napa_core.c at 8e8d1fe).
//
// Codes are 36 bits, most significant bit first in the family's cell order
// (AprilTag's bit_x/bit_y). Every code is stored in its 4 rotations, at index
// rotation * 587 + id, where rotation r is the code turned 90 degrees
// counter-clockwise r times (numpy's rot90: out[y][x] = in[x][grid-1-y]).
//
// Multi-index hashing: a read within 3 bits of a code matches at least one of
// the code's four 9-bit blocks exactly (pigeonhole). So four 512-entry tables,
// one per block, each list the rotated codes with that block value, and only
// those candidates are compared. In hardware that's four block-RAM lookups and
// a short candidate list. bucket sizes below give the list length to budget.
//
// tag36h11 codes are at least 11 bits apart, so a code within 3 bits of the
// read is the only one. A 3-bit correction is accepted only when the wrong
// cells form one 8-connected group (real damage like a finger or glare spot is
// spatially coherent; random patterns that land within 3 bits are scattered).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace gv::model {

struct TagMatch {
  int id = 0;
  int rotation = 0;  // the read equals the canonical code rotated this many times
  int hamming = 0;   // corrected bits
  bool operator==(const TagMatch&) const = default;
};

class Tag36h11Lookup {
 public:
  static constexpr int kBits = 36;
  static constexpr int kBlocks = 4;
  static constexpr int kBlockBits = 9;
  static constexpr int kDefaultMaxBits = 3;

  Tag36h11Lookup();

  int num_codes() const { return static_cast<int>(canonical_.size()); }
  int num_rotated() const { return static_cast<int>(rotated_.size()); }
  uint64_t canonical(int id) const { return canonical_[static_cast<size_t>(id)]; }
  uint64_t rotated(int index) const { return rotated_[static_cast<size_t>(index)]; }

  // The code turned 90 degrees counter-clockwise.
  uint64_t rotate(uint64_t code) const;

  // NAPA's nearest_code() for max_bits <= 3: the rotated-code index within
  // max_bits of the read, or -1. Searches only the hashed candidates.
  int nearest(uint64_t read, int max_bits, int* distance) const;

  // The full decision: nearest(), then the 8-connected rule for 3-bit fixes.
  std::optional<TagMatch> decode(uint64_t read, int max_bits = kDefaultMaxBits) const;

  // Whether the set bits of diff (as cells) form one 8-connected group.
  bool errors_connected(uint64_t diff) const;

  // Candidate list for one block value, in table order.
  const std::vector<uint32_t>& bucket(int block, uint32_t value) const {
    return buckets_[static_cast<size_t>(block)][value];
  }
  size_t max_bucket_size() const;

  // Each bit's cell, in data-grid coordinates (AprilTag's bit_x, bit_y).
  int cell_x(int bit) const { return cell_x_[static_cast<size_t>(bit)]; }
  int cell_y(int bit) const { return cell_y_[static_cast<size_t>(bit)]; }

 private:
  std::vector<uint64_t> canonical_;
  std::vector<uint64_t> rotated_;
  std::array<int, kBits> cell_x_{}, cell_y_{};
  std::array<int, kBits> rot_src_{};    // bit k of the rotated code comes from this bit
  std::array<uint64_t, kBits> near_{};  // bits whose cells touch bit k's cell (8-connected)
  std::array<std::vector<std::vector<uint32_t>>, kBlocks> buckets_;
};

}  // namespace gv::model
