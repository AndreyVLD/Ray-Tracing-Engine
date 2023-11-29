#include "extra.h"
#include "bvh.h"
#include "draw.h"
#include "light.h"
#include "recursive.h"
#include "shading.h"
#include "texture.h"
#include <framework/trackball.h>
#include <cmath>
#include <algorithm>
#include <iostream>

int numsamplesDOF = 10;
float focalLength = 3.0f;
float lensSize = 0.4f;

int filterSize = 9;
float scale = 1.2f;
float threshold = 0.6f;

int numSampleMotionBlur = 25;

glm::vec3 computeFocusPoint(const int x, const int y, const Trackball& camera, const glm::vec2& resolution)
{
    // Generate a ray for each pixel through the middle of the pixel
    glm::vec2 position = (glm::vec2(x, y) + 0.5f) / resolution * 2.f - 1.f;
    Ray cameraRay = camera.generateRay(position);

    // Compute the focus point for that pixel
    glm::vec3 focusPoint = camera.position() + focalLength * glm::normalize(cameraRay.direction);

    return focusPoint;
}

// TODO; Extra feature
// Given the same input as for `renderImage()`, instead render an image with your own implementation
// of Depth of Field. Here, you generate camera rays s.t. a focus point and a thin lens camera model
// are in play, allowing objects to be in and out of focus.
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void renderImageWithDepthOfField(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
{
    if (!features.extra.enableDepthOfField) {
        return;
    }
    const int screenWidth = screen.resolution().x;
    const int screenHeight = screen.resolution().y;

    //Iterate over all pixels
    for (int y = 0; y < screenHeight; y++) {
        for (int x = 0; x < screenWidth; x++) {
            RenderState state = {
                .scene = scene,
                .features = features,
                .bvh = bvh,
                .sampler = { static_cast<uint32_t>(screenHeight * x + y) }
            };

            // Compute the basis of the camera plane
            glm::vec3 basis1 = glm::normalize(camera.up()) * lensSize / 2.0f;
            glm::vec3 basis2 = glm::normalize(camera.left()) * lensSize / 2.0f;
      
            // Compute the focus point for that pixel
            glm::vec3 focusPoint = computeFocusPoint(x, y, camera, glm::vec2(screen.resolution()));

            glm::vec3 contribution(0.0f);

            // Generate a ray for each sample
            for (int i = 0; i < numsamplesDOF; i++) {

                // Generate a random point on the lens
                float x1 = (state.sampler.next_1d() - 0.5f) * 2;
                float x2 = (state.sampler.next_1d() - 0.5f) * 2;

                // Origin of the ray starting somewhere on the lens
                glm::vec3 rayOrigin = camera.position() + x1 * basis1 + x2 * basis2;

                // Compute the direction of the ray
                glm::vec3 rayDirection = glm::normalize(focusPoint - rayOrigin);

                Ray ray = { rayOrigin, rayDirection };

                auto L = renderRay(state, ray);
                // Accumulate the contribution
                contribution += L;
            }

            // Average the contribution to create the blur effect for out of focus objects
            contribution = contribution / static_cast<float> (numsamplesDOF);
            screen.setPixel(x, y, contribution);
        }
    }
}

glm::vec3 positionAtTime(const float t) {
    // Bezier curve points
    std::array<glm::vec3, 4> control = {
        glm::vec3 { 0, 1, 1 }, glm::vec3 { -1, 2, 1 }, glm::vec3 { 1, 3, 2 }, glm::vec3 { -1, 0, 0 }
    };

    float coef1 = (1 - t) * (1 - t) * (1 - t);
    float coef2 = 3.0f * (1 - t) * (1 - t) * t;
    float coef3 = 3.0f * (1 - t) * t * t;
    float coef4 = t * t * t;

    glm::vec3 bezierPoint = coef1 * control[0] + coef2 * control[1] + coef3 * control[2] + coef4 * control[3];
    return bezierPoint;
}

// TODO; Extra feature
// Given the same input as for `renderImage()`, instead render an image with your own implementation
// of motion blur. Here, you integrate over a time domain, and not just the pixel's image domain,
// to give objects the appearance of "fast movement".
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void renderImageWithMotionBlur(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
{
    if (!features.extra.enableMotionBlur) {
        return;
    }

    int width = screen.resolution().x;
    int height = screen.resolution().y;
    std::vector<glm::vec3> pixels(width * height);

    // Copy the initial image
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = screen.indexAt(x, y);
            pixels[index] = screen.pixels()[index];
        }
    }

    // Render the image for each time step
    for (int i = 0; i < numSampleMotionBlur; i++) {
        Scene sceneNew = Scene(scene);

        // Move the objects according to the Bezier curve
        for (Mesh& mesh : sceneNew.meshes) {
            for (Vertex& v : mesh.vertices) {
                float t = static_cast<float>(i) / numSampleMotionBlur;

                glm::vec3 bezierPoint = positionAtTime(t);
                v.position += bezierPoint / 10.0f;
            }

		}
        // Create a new BVH for each time step
        BVH newBVH =  BVH::BVH(sceneNew, features);

        // Render the rays for each time step
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                RenderState state = {
                    .scene = sceneNew,
                    .features = features,
                    .bvh = newBVH,
                    .sampler = { static_cast<uint32_t>(height * i + j) }
                };
                int t = screen.indexAt(i, j);
                auto rays = generatePixelRays(state, camera, glm::vec2(i, j), screen.resolution());
                pixels[t] += renderRays(state, rays);
            }
        }
    }

    // Average the image over all time steps
    for (int i = 0; i < width; i++)
        for (int j = 0; j < height; j++) {
            int t = screen.indexAt(i, j);
            screen.setPixel(i, j, pixels[t] * (1.0f / numSampleMotionBlur));
		}
}


//create 1D filter based on filterSize
std::vector<float> createFilter() {
    float value = 1.0f;
    float acc = 1.0f;
    std::vector<float> filter;
    filter.push_back(value);

    for (int i = 1; i < filterSize; i++) {
        value = value * (filterSize - i + 1) / (1.0f * i);
        acc += value;
        filter.push_back(value);
    }

    for (int i = 0; i < filterSize; i++) {
        filter[i] /= acc;
    }

    return filter;
}

Screen applyHorizontalFilter(const Screen& image, const std::vector<float>& filter) {
    Screen temp = Screen(image);

    for (int i = 0; i < image.resolution().y; i++)
        for (int j = 0; j < image.resolution().x; j++) {
			glm::vec3 color(0.0f);

            for (int k = 0; k < filterSize; k++) {
				int centered = j - filterSize / 2 + k; // considering j as the center of the filter
                if (centered >= 0 && centered < image.resolution().x) {
					float filterValue = filter[k];
                    if (filterValue != 0.0f) {
						color += image.pixels()[image.indexAt(centered, i)] * filterValue;
					}
				}

			}
			temp.setPixel(j, i, color);
		}

    return temp;
}

Screen applyVerticalFilter(const Screen& image, const std::vector<float>& filter)
{
    Screen temp = Screen(image);

    for (int i = 0; i < image.resolution().y; i++)
		for (int j = 0; j < image.resolution().x; j++) {
            glm::vec3 color(0.0f);

            for (int k = 0; k < filterSize; k++) {
                int centered = i - filterSize / 2 + k; // considering i as the center of the filter
                if (centered >= 0 && centered < image.resolution().y) {
					float filterValue = filter[k];
                    if (filterValue != 0.0f) {
						color += image.pixels()[image.indexAt(j, centered)] * filterValue;
					}
				}
            }
            temp.setPixel(j, i, color);
        }
    return temp;
}

void applyThreshold(const Screen& source,Screen &temp) 
{
    for (int y = 0; y < source.resolution().y; y++) {
        for (int x = 0; x < source.resolution().x; x++) {
            glm::vec3 color = source.pixels()[source.indexAt(x, y)];

            // if a pixel exceeds a certain threshold, we apply the filter
            if (color.x > threshold || color.y > threshold || color.z > threshold)
                temp.setPixel(x, y, color);
            else
                color = glm::vec3(0.0f); 
        }
    }

}

    // TODO; Extra feature
// Given a rendered image, compute and apply a bloom post-processing effect to increase bright areas.
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void postprocessImageWithBloom(const Scene& scene, const Features& features, const Trackball& camera, Screen& image)
{
    if (!features.extra.enableBloomEffect) {
        return;
    }

    Screen temp = Screen(image.resolution());

    // Make fitler size odd so that we can have a center
    filterSize = filterSize - 1 + filterSize % 2;
    std::vector<float> filter = createFilter();

    // 1. Apply threshold to the image and copy to temp
    applyThreshold(image, temp);

    // 2. Apply horizontal filter to the thresholded image

    Screen appliedHorizontal = applyHorizontalFilter(temp, filter);

    // 3. Apply vertical filter to the horizontal filtered image
    Screen final = applyVerticalFilter(appliedHorizontal, filter);

    // 4. Scale the filtered image and add it to the initial image
    temp = Screen(image.resolution());

    // Adding the blured image to the initial image
    for (int i = 0; i < temp.resolution().x; i++) {
        for (int j = 0; j < temp.resolution().y; j++) {

            glm::vec3 color = final.pixels()[final.indexAt(i, j)] + scale * image.pixels()[image.indexAt(i, j)];
            temp.setPixel(i, j, color);
        }
    }
    image = temp;
}




// TODO; Extra feature
// Given a camera ray (or reflected camera ray) and an intersection, evaluates the contribution of a set of
// glossy reflective rays, recursively evaluating renderRay(..., depth + 1) along each ray, and adding the
// results times material.ks to the current intersection's hit color.
// - state;    the active scene, feature config, bvh, and sampler
// - ray;      camera ray
// - hitInfo;  intersection object
// - hitColor; current color at the current intersection, which this function modifies
// - rayDepth; current recursive ray depth
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void renderRayGlossyComponent(RenderState& state, Ray ray, const HitInfo& hitInfo, glm::vec3& hitColor, int rayDepth)
{
    // number of samples to be generated
    auto numSamples = state.features.extra.numGlossySamples;

    // generate initial specular ray
    Ray reflection = generateReflectionRay(ray, hitInfo);

    // construct an orthonormal basis from the direction of the ray 
    glm::vec3 u = glm::normalize(reflection.direction), v, w = glm::normalize(reflection.direction);

    if (abs(u[0]) <= abs(u[1]) && abs(u[0]) <= abs(u[2]))
        u[0] = 1.0f;
    else if (abs(u[1]) <= abs(u[2]) && abs(u[1]) <= abs(u[0]))
        u[1] = 1.0f;
    else
        u[2] = 1.0f;

    u = glm::normalize(glm::cross(u, w));
    v = glm::normalize(glm::cross(u, w));

    
    Sampler sampler;

    // generate glossy reflective rays
    for (int i = 0; i < numSamples; ++i) {
        // generate a point on the unit disk and make the radius shininess / 64
        auto point = sampler.next_circle() * (hitInfo.material.shininess / 64.f);
        // transform point to a direction
        auto direction = glm::normalize(reflection.direction + point[0] * u + point[1] * v);
        // ray with proper direction
        Ray glossyRay { .origin = reflection.origin + 0.001f * direction, .direction = direction };
        
        glm::vec3 contribution;
        
        // if sampled ray is behind the object, then its contribution should be 0
        if (glm::dot(glossyRay.direction, hitInfo.normal) < 0.0f)
            contribution = glm::vec3(0.0f);
        else
            // get contribution of glossy ray
            contribution = renderRay(state, glossyRay, rayDepth + 1);
        
        // add the contribution to the result
        hitColor += contribution * hitInfo.material.ks;
    }

    // divide the contribution by the number of samples
    hitColor /= numSamples;
}



// TODO; Extra feature
// Given a camera ray (or reflected camera ray) that does not intersect the scene, evaluates the contribution
// along the ray, originating from an environment map. You will have to add support for environment textures
// to the Scene object, and provide a scene with the right data to supply this.
// - state; the active scene, feature config, bvh, and sampler
// - ray;   ray object
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!


// Helper methods for computing texture coordinates
inline glm::vec2 sphereMap(glm::vec3 direction)
{
    return glm::vec2(
        glm::asin(direction.x) / glm::pi<float>() + 0.5f,
        glm::asin(direction.y) / glm::pi<float>() + 0.5f);
}

/*
    
*/

glm::vec2 cubeMapping(glm::vec3 direction) {
    float mxCoord = glm::max(std::abs(direction.x), glm::max(std::abs(direction.y), std::abs(direction.z)));
    direction /= mxCoord;
    glm::vec2 scale = glm::vec2(1.0f / 4.0f, 1.0f / 3.0f);

    glm::vec2 coords = glm::vec2(0.0f);
    // case 1: x > 0 and |x| = mxCoord
    if (std::abs(direction.x - 1.0f) < 1e-6) {
        coords = 0.5f * glm::vec2(1.0f + direction.z, 1.0f + direction.y);
        coords = scale * coords + glm::vec2(0.5f, 1.0f / 3.0f);
    }
    // case 2: x < 0 and |x| = mxCoord
    if (std::abs(direction.x + 1.0f) < 1e-6) {
        coords = 0.5f * glm::vec2(1.0f - direction.z, 1.0f + direction.y);
        coords = scale * coords + glm::vec2(0.0f, 1.0f / 3.0f);
    }
    // case 3: y > 0 and |y| = mxCoord
    if (std::abs(direction.y + 1.0f) < 1e-6) {
        coords = 0.5f * glm::vec2(1.0f + direction.x, 1.0f - direction.z);
        coords = scale * coords + glm::vec2(0.25f, 0.0f);
    }
    // case 4: y < 0 and |y| = mxCoord
    if (std::abs(direction.y - 1.0f) < 1e-6) {
        coords = 0.5f * glm::vec2(1.0f + direction.x, 1.0f + direction.z);
        coords = scale * coords + glm::vec2(0.25f, 2.0f / 3.0f);
    } 
    // case 5: z > 0 and |z| = mxCoord
    if (std::abs(direction.z + 1.0f) < 1e-6) {
        coords = 0.5f * glm::vec2(1.0f + direction.x, 1.0f + direction.y);
        coords = scale * coords + glm::vec2(0.25f, 1.0f / 3.0f);
    }
    // case 6: z < 0 and |z| = mxCoord
    if (std::abs(direction.z - 1.0f) < 1e-6) {
        coords = 0.5f * glm::vec2(1.0f - direction.x, 1.0f + direction.y);
        coords = scale * coords + glm::vec2(0.75f, 1.0f / 3.0f);
    }
    
    return coords;
}


glm::vec3 sampleEnvironmentMap(RenderState& state, Ray ray)
{
    if (state.features.extra.enableEnvironmentMap) {
        

        // Part of your implementation should go here
        glm::vec2 texCoords = cubeMapping(glm::normalize(ray.direction));
        assert(0.0f <= texCoords.x && texCoords.x <= 1.0f);
        assert(0.0f <= texCoords.y && texCoords.y <= 1.0f);
        return sampleTextureNearest(state.scene.envMap, texCoords);
    }
    else {
        return glm::vec3(0.f);
    }
}


void updateAABB(AxisAlignedBox& aabb, BVHInterface::Primitive prim) {
    const auto& [v0, v1, v2] = std::tie(prim.v0, prim.v1, prim.v2);
    aabb.lower = glm::min(aabb.lower, glm::min(v0.position, glm::min(v1.position, v2.position)));
    aabb.upper = glm::max(aabb.upper, glm::max(v0.position, glm::max(v1.position, v2.position)));
}

void updateAABB(AxisAlignedBox& box1, AxisAlignedBox& box2) {
    box1.lower = glm::min(box1.lower, box2.lower);
    box1.upper = glm::max(box1.upper, box2.upper);
}

inline float computeSurfaceArea(AxisAlignedBox& box) {
    glm::vec3 sides = box.upper - box.lower;
    return (sides[0] * sides[1] + sides[1] * sides[2] + sides[2] * sides[0]);
}

inline float computeCost(float leftArea, uint32_t leftCount, float rightArea, uint32_t rightCount) {
    if (leftCount == 0 || rightCount == 0) {
        return std::numeric_limits<float>::max();
    }
    return leftArea * leftCount + rightArea * rightCount;
}

void splitIntoBins(std::vector<AxisAlignedBox>& bins, std::vector<uint32_t>& leftCount, std::vector<uint32_t>& rightCount, std::vector<float>& leftArea, std::vector<float>& rightArea, std::span<BVH::Primitive> primitives, uint32_t axis, const AxisAlignedBox& aabb) {
    float regionLength = BVH::Bins / (aabb.upper[axis] - aabb.lower[axis]);

    for (int i = 0; i < primitives.size(); ++i) {
        float centroid = computePrimitiveCentroid(primitives[i])[axis];
        int32_t region = std::min((int32_t)BVH::Bins - 1, (int32_t)((centroid - aabb.lower[axis]) * regionLength));

        leftCount[region]++;
        rightCount[region]++;

        updateAABB(bins[region], primitives[i]);
    }

    // compute leftCount
    for (int i = 1; i < BVH::Bins; ++i)
        leftCount[i] += leftCount[i - 1];

    // compute rightCount
    for (int i = BVH::Bins - 2; i >= 0; --i) {
        rightCount[i] += rightCount[i + 1];
    }

    // compute leftArea
    AxisAlignedBox currAABB = { .lower = glm::vec3(std::numeric_limits<float>::max()), .upper = glm::vec3(-std::numeric_limits<float>::max()) };
    for (int i = 0; i < BVH::Bins; ++i) {
        updateAABB(currAABB, bins[i]);
        leftArea[i] = computeSurfaceArea(currAABB);
    }

    // compute rightArea
    currAABB = { .lower = glm::vec3(std::numeric_limits<float>::max()),
        .upper = glm::vec3(-std::numeric_limits<float>::max()) };
    for (int i = BVH::Bins - 1; i >= 0; --i) {
        updateAABB(currAABB, bins[i]);
        rightArea[i] = computeSurfaceArea(currAABB);
    }
}

void getMinimalCost(std::vector<uint32_t>& leftCount, std::vector<uint32_t>& rightCount, std::vector<float>& leftArea, std::vector<float>& rightArea, float& minCost, uint32_t& currIdx) {
    // find minimal cost
    minCost = std::numeric_limits<float>::max();
    currIdx = 0;
    
    for (size_t i = 0; i < BVH::Bins - 1; ++i) {
        float currCost = computeCost(leftArea[i], leftCount[i], rightArea[i + 1], rightCount[i + 1]);
        if (currCost < minCost) {
            minCost = currCost;
            currIdx = i;
        }
    }
}

// TODO: Extra feature
// As an alternative to `splitPrimitivesByMedian`, use a SAH+binning splitting criterion. Refer to
// the `Data Structures` lecture for details on this metric.
// - aabb;       the axis-aligned bounding box around the given triangle set
// - axis;       0, 1, or 2, determining on which axis (x, y, or z) the split must happen
// - primitives; the modifiable range of triangles that requires splitting
// - return;     the split position of the modified range of triangles
// This method is unit-tested, so do not change the function signature.
size_t splitPrimitivesBySAHBin(const AxisAlignedBox& aabb, uint32_t axis, std::span<BVH::Primitive> primitives)
{
    using Primitive = BVH::Primitive;


    float regionLength = BVH::Bins / (aabb.upper[axis] - aabb.lower[axis]);
    std::vector<AxisAlignedBox> bins(BVH::Bins, { .lower = glm::vec3(std::numeric_limits<float>::max()), .upper = glm::vec3(-std::numeric_limits<float>::max()) });
    std::vector<uint32_t> leftCount(BVH::Bins), rightCount(BVH::Bins);
    std::vector<float> leftArea(BVH::Bins, std::numeric_limits<float>::max()), rightArea(BVH::Bins, std::numeric_limits<float>::max());

    splitIntoBins(bins, leftCount, rightCount, leftArea, rightArea, primitives, axis, aabb);

    // find minimal cost
    float minCost = -std::numeric_limits<float>::max();
    uint32_t currIdx = 0;
    getMinimalCost(leftCount, rightCount, leftArea, rightArea, minCost, currIdx);

    // rearrange span
    size_t currI = 0;
    for (size_t i = 0; i < primitives.size(); ++i) {
        float centroid = computePrimitiveCentroid(primitives[i])[axis];
        int32_t region = std::min((int32_t)BVH::Bins - 1, (int32_t)((centroid - aabb.lower[axis]) * regionLength));
        assert(region >= 0);
        if (region <= currIdx) {
            std::swap(primitives[i], primitives[currI++]);
        }
    }

    return currI - 1;
}