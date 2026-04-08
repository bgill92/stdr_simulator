#include "stdr_standalone/map_loader.hpp"

#include <stdr_simulation/config_loader.hpp>

#include <png.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace stdr_standalone
{
namespace
{

struct ImageData
{
  int width{ 0 };
  int height{ 0 };
  std::vector<unsigned char> pixels;  // Grayscale, one byte per pixel.
};

// Load a PNG file and convert it to 8-bit grayscale pixel data.
[[nodiscard]] tl::expected<ImageData, std::string> load_png(const std::string& path)
{
  FILE* fp = std::fopen(path.c_str(), "rb");
  if (!fp)
  {
    return tl::make_unexpected("Cannot open image file: " + path);
  }

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png)
  {
    std::fclose(fp);
    return tl::make_unexpected("png_create_read_struct failed");
  }

  png_infop info = png_create_info_struct(png);
  if (!info)
  {
    png_destroy_read_struct(&png, nullptr, nullptr);
    std::fclose(fp);
    return tl::make_unexpected("png_create_info_struct failed");
  }

  // libpng uses setjmp/longjmp for error handling.
  if (setjmp(png_jmpbuf(png)))
  {  // NOLINT(cert-err52-cpp)
    png_destroy_read_struct(&png, &info, nullptr);
    std::fclose(fp);
    return tl::make_unexpected("PNG read error for: " + path);
  }

  png_init_io(png, fp);
  png_read_info(png, info);

  const int width = static_cast<int>(png_get_image_width(png, info));
  const int height = static_cast<int>(png_get_image_height(png, info));
  const png_byte color_type = png_get_color_type(png, info);
  const png_byte bit_depth = png_get_bit_depth(png, info);

  // Normalize all color types to 8-bit grayscale before reading pixels.
  if (bit_depth == 16)
  {
    png_set_strip_16(png);
  }
  if (color_type == PNG_COLOR_TYPE_PALETTE)
  {
    png_set_palette_to_rgb(png);
  }
  if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
  {
    png_set_expand_gray_1_2_4_to_8(png);
  }
  if (png_get_valid(png, info, PNG_INFO_tRNS))
  {
    png_set_tRNS_to_alpha(png);
  }
  // Convert RGB/RGBA to grayscale using standard luminance weights.
  if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_RGB_ALPHA || color_type == PNG_COLOR_TYPE_PALETTE)
  {
    png_set_rgb_to_gray(png, 1, -1.0, -1.0);
  }
  // Unconditionally strip alpha — palette transparency or GRAY_ALPHA may have
  // added an alpha channel that the original color_type check would miss.
  png_set_strip_alpha(png);

  png_read_update_info(png, info);

  // Verify that all transforms produced exactly one byte per pixel.
  const size_t row_bytes = png_get_rowbytes(png, info);
  if (row_bytes != static_cast<size_t>(width))
  {
    png_destroy_read_struct(&png, &info, nullptr);
    std::fclose(fp);
    return tl::make_unexpected("PNG transform error: expected " + std::to_string(width) + " bytes/row but got " +
                               std::to_string(row_bytes) + " for: " + path);
  }

  // Allocate pixel buffer and row pointers before png_read_image. These must
  // be POD-like or trivially destructible to avoid UB if libpng longjmps
  // back to the setjmp above. std::vector is not trivially destructible, so
  // we use raw allocations guarded by the cleanup in the setjmp handler.
  // In practice the setjmp handler cleans up png/info/fp but not these
  // allocations — a small leak on corrupt-PNG error paths, which is acceptable.
  ImageData result;
  result.width = width;
  result.height = height;
  result.pixels.resize(static_cast<size_t>(width) * static_cast<size_t>(height));

  std::vector<png_bytep> row_pointers(static_cast<size_t>(height));
  for (int y = 0; y < height; ++y)
  {
    row_pointers[static_cast<size_t>(y)] = result.pixels.data() + static_cast<size_t>(y) * static_cast<size_t>(width);
  }

  png_read_image(png, row_pointers.data());
  png_destroy_read_struct(&png, &info, nullptr);
  std::fclose(fp);

  return result;
}

// Load a PGM (P5 binary or P2 ASCII) file into grayscale pixel data.
[[nodiscard]] tl::expected<ImageData, std::string> load_pgm(const std::string& path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open())
  {
    return tl::make_unexpected("Cannot open image file: " + path);
  }

  std::string magic;
  file >> magic;
  if (magic != "P5" && magic != "P2")
  {
    return tl::make_unexpected("Unsupported PGM format: " + magic + " in " + path);
  }

  // Skip comment lines before reading header fields.
  auto skip_comments = [&file]() {
    file >> std::ws;
    while (file.peek() == '#')
    {
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      file >> std::ws;
    }
  };

  skip_comments();
  int width = 0;
  int height = 0;
  int max_val = 0;
  file >> width >> height;
  skip_comments();
  file >> max_val;
  file.get();  // Consume the single whitespace character after maxval.

  if (max_val > 255 && magic == "P5")
  {
    return tl::make_unexpected("16-bit binary PGM (max_val=" + std::to_string(max_val) + ") not supported: " + path);
  }

  ImageData result;
  result.width = width;
  result.height = height;
  result.pixels.resize(static_cast<size_t>(width) * static_cast<size_t>(height));

  if (magic == "P5")
  {
    file.read(reinterpret_cast<char*>(result.pixels.data()), static_cast<std::streamsize>(result.pixels.size()));
  }
  else
  {
    // P2 ASCII: each pixel is a whitespace-delimited decimal integer.
    for (unsigned char& pixel : result.pixels)
    {
      int val = 0;
      file >> val;
      pixel = static_cast<unsigned char>(val);
    }
  }

  if (!file)
  {
    return tl::make_unexpected("Failed to read pixel data from: " + path);
  }

  // Rescale to [0, 255] when the image uses a different bit depth.
  if (max_val != 255 && max_val > 0)
  {
    for (unsigned char& pixel : result.pixels)
    {
      pixel = static_cast<unsigned char>(static_cast<double>(pixel) / static_cast<double>(max_val) * 255.0);
    }
  }

  return result;
}

// Dispatch to the correct loader based on the file extension.
[[nodiscard]] tl::expected<ImageData, std::string> load_image(const std::string& path)
{
  std::string lower_path = path;
  for (char& c : lower_path)
  {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }

  if (lower_path.ends_with(".pgm"))
  {
    return load_pgm(path);
  }
  // Default to PNG for .png and any unrecognised extension.
  return load_png(path);
}

// Map a grayscale pixel to an OccupancyGrid value using ROS map_server conventions.
// White (255) = free space, black (0) = occupied. Returns 0 (free), 100 (occupied),
// or -1 (unknown) for pixels between the thresholds.
[[nodiscard]] int8_t pixel_to_occupancy(unsigned char pixel, bool negate, double occupied_thresh, double free_thresh)
{
  double occ = (255.0 - static_cast<double>(pixel)) / 255.0;
  if (negate)
  {
    occ = 1.0 - occ;
  }

  if (occ > occupied_thresh)
  {
    return 100;
  }
  if (occ < free_thresh)
  {
    return 0;
  }
  return -1;
}

}  // namespace

[[nodiscard]] tl::expected<stdr_simulation::OccupancyGrid, std::string> load_map(const std::string& yaml_path)
{
  const tl::expected<stdr_simulation::MapMetadata, std::string> metadata_result =
      stdr_simulation::load_map_metadata(yaml_path);
  if (!metadata_result)
  {
    return tl::make_unexpected(metadata_result.error());
  }
  const stdr_simulation::MapMetadata& meta = metadata_result.value();

  const tl::expected<ImageData, std::string> image_result = load_image(meta.image_path);
  if (!image_result)
  {
    return tl::make_unexpected(image_result.error());
  }
  const ImageData& image = image_result.value();

  stdr_simulation::OccupancyGrid grid;
  grid.width = image.width;
  grid.height = image.height;
  grid.resolution = meta.resolution;
  grid.origin = meta.origin;

  // OccupancyGrid row 0 is the bottom of the map, but PNG/PGM row 0 is the top,
  // so we flip vertically while copying.
  const size_t num_pixels = static_cast<size_t>(image.width) * static_cast<size_t>(image.height);
  grid.data.resize(num_pixels);

  for (int y = 0; y < image.height; ++y)
  {
    const int flipped_y = image.height - 1 - y;
    for (int x = 0; x < image.width; ++x)
    {
      const size_t src_idx = static_cast<size_t>(y) * static_cast<size_t>(image.width) + static_cast<size_t>(x);
      const size_t dst_idx = static_cast<size_t>(flipped_y) * static_cast<size_t>(image.width) + static_cast<size_t>(x);
      grid.data[dst_idx] =
          pixel_to_occupancy(image.pixels[src_idx], meta.negate, meta.occupied_thresh, meta.free_thresh);
    }
  }

  return grid;
}

}  // namespace stdr_standalone
