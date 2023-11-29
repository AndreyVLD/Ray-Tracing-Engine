#include "texture.h"
#include "render.h"
#include <framework/image.h>
#include <iostream>

// TODO: Standard feature
// Given an image, and relevant texture coordinates, sample the texture s.t.
// the nearest texel to the coordinates is acquired from the image.
// - image;    the image object to sample from.
// - texCoord; sample coordinates, generally in [0, 1]
// - return;   the nearest corresponding texel
// This method is unit-tested, so do not change the function signature.
glm::vec3 sampleTextureNearest(const Image& image, const glm::vec2& texCoord)
{
    // TODO: implement this function.
    // Note: the pixels are stored in a 1D array, row-major order. You can convert from (i, j) to
    //       an index using the method seen in the lecture.
    // Note: the center of the first pixel should be at coordinates (0.5, 0.5)
    // Given texcoords, return the corresponding pixel of the image
    // The imagine is fliped in x axis
    if (texCoord.x < 0.0f || texCoord.x > 1.0f || texCoord.y < 0.0f || texCoord.y > 1.0f) {
		return glm::vec3(0.0f);
	}
    float i = texCoord.x * image.width;
    float j = image.height - texCoord.y * image.height;

    i = glm::clamp(i, 0.0f, (float)image.width);
    j = glm::clamp(j, 0.0f, (float)image.height);

    int x = (int)glm::round(i-0.5);
    int y = (int)glm::round(j-0.5);
  
    int index = y * image.width + x;
   
    glm::vec3 pixel = image.pixels[index];
   
    return pixel;
 
}

// TODO: Standard feature
// Given an image, and relevant texture coordinates, sample the texture s.t.
// a bilinearly interpolated texel is acquired from the image.
// - image;    the image object to sample from.
// - texCoord; sample coordinates, generally in [0, 1]
// - return;   the filter of the corresponding texels
// This method is unit-tested, so do not change the function signature.
glm::vec3 sampleTextureBilinear(const Image& image, const glm::vec2& texCoord)
{
    // TODO: implement this function.
    // Note: the pixels are stored in a 1D array, row-major order. You can convert from (i, j) to
    //       an index using the method seen in the lecture.
    // Note: the center of the first pixel should be at coordinates (0.5, 0.5)
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    glm::vec2 clampedTexCoord = glm::clamp(texCoord, glm::vec2(0.0f), glm::vec2(1.0f));


    // Compute the position of the pixel in the image
    float x = clampedTexCoord.x * image.width;
    float y = image.height - clampedTexCoord.y * image.height;
    // Compute the position of the 4 pixels around the pixel
    int x0 = (int)std::floor(x-0.5);
    int y0 = (int)std::floor(y-0.5);
    int x1 = (int)std::ceil(x-0.5);
    int y1 = (int)std::ceil(y-0.5);
    // Compute the distance between the pixel and the 4 pixels around it
    float dx = x  - x0 - 0.5;
    float dy = y  - y0 - 0.5;
    x0 = glm::clamp(x0, 0, image.width - 1);
    y0 = glm::clamp(y0, 0, image.height - 1);
    x1 = glm::clamp(x1, 0, image.width - 1);
    y1 = glm::clamp(y1, 0, image.height - 1);
    // Compute the index of the 4 pixels around the pixel
    int index00 = y0 * image.width + x0;
    int index01 = y0 * image.width + x1;
    int index10 = y1 * image.width + x0;
    int index11 = y1 * image.width + x1;
    // Compute the pixel value of the 4 pixels around the pixel
    glm::vec3 pixel00 = image.pixels[index00];
    glm::vec3 pixel01 = image.pixels[index01];
    glm::vec3 pixel10 = image.pixels[index10];
    glm::vec3 pixel11 = image.pixels[index11];
    // Interpolate the pixel value
    glm::vec3 pixel0 = (1.0f - dx) * pixel00 + dx * pixel01;
    glm::vec3 pixel1 = (1.0f - dx) * pixel10 + dx * pixel11;
    glm::vec3 pixel = (1.0f - dy) * pixel0 + dy * pixel1;
    return pixel;
}