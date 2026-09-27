#include "jpeg_decode.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO  // frames come from memory
#include "stb_image.h"

bool DecodeJpeg(const std::vector<unsigned char>& jpg, std::vector<unsigned char>& rgb, int& w, int& h) {
  if (jpg.empty() || jpg.size() > 0x7fffffffu) return false;
  int jw = 0, jh = 0, channels = 0;
  unsigned char* px = stbi_load_from_memory(jpg.data(), static_cast<int>(jpg.size()), &jw, &jh, &channels, 3);
  if (!px) return false;
  rgb.assign(px, px + static_cast<size_t>(jw) * static_cast<size_t>(jh) * 3);
  stbi_image_free(px);
  w = jw;
  h = jh;
  return true;
}
