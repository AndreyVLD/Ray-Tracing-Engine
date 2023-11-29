// Put your includes here
#include "bvh.h"
#include "render.h"
#include "sampler.h"
#include "scene.h"
#include "shading.h"
#include <limits>
#include <iostream>

// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <catch2/catch_all.hpp>
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()

// In this file you can add your own unit tests using the Catch2 library.
// You can find the documentation of Catch2 at the following link:
// https://github.com/catchorg/Catch2/blob/devel/docs/assertions.md
//
// These tests are only to help you verify that your code is correct.
// You don't have to hand them in; we will not consider them when grading.
//

// Add your tests here, if you want :D
TEST_CASE("StudentTest")
{
    Features features = {
        .enableShading = true,
        .enableAccelStructure = true,
        .shadingModel = ShadingModel::Lambertian
    };

    Scene scene = loadScenePrebuilt(SceneType::SingleTriangle, DATA_DIR);
    BVH bvh(scene, features);
    RenderState state = {
        .scene = scene,
        .features = features,
        .bvh = bvh,
        .sampler = {}
    };
    
    BVHInterface::Primitive prim = {
        .v0 = { .position = glm::vec3(0.0f, 1.0f, 0.0f) },
        .v1 = { .position = glm::vec3(1.0f, -1.0f, 0.0f) },
        .v2 = { .position = glm::vec3(-2.0f, 0.5f, 1.0f) }
    };

    SECTION("AABB generation") {
        CHECK(computePrimitiveAABB(prim).lower == glm::vec3(-2.0f, -1.0f, 0.0f));
        CHECK(computePrimitiveAABB(prim).upper == glm::vec3(1.0f, 1.0f, 1.0f));

    }

    SECTION("AABB longest axis") {
        AxisAlignedBox box = {
            .lower = glm::vec3(0.0f),
            .upper = glm::vec3(1.0f)
        };
        CHECK(computeAABBLongestAxis(box) == 0);
        CHECK(computeAABBLongestAxis(AxisAlignedBox {
                  .lower = glm::vec3(0.0f, -1.0f, 2.0f),
                  .upper = glm::vec3(1.0f, 0.0f, 5.0f) })
            == 2);
        CHECK(computeAABBLongestAxis({ .lower = glm::vec3(0.0f, -1.0f, 2.0f),
                  .upper = glm::vec3(0.0f, 2.0f, 5.0f) })
            == 1);
    }

    BVHInterface::Primitive prim1 = {
        .v0 = { .position = glm::vec3(0.0f, 1.0f, 0.0f) },
        .v1 = { .position = glm::vec3(1.0f, 1.0f, 0.0f) },
        .v2 = { .position = glm::vec3(2.0f, 1.0f, 3.0f) }
    };

    SECTION("Primitive centroid") {
        CHECK(computePrimitiveCentroid(prim1) == glm::vec3(1.0f, 1.0f, 1.0f));
    }

    SECTION("BVH generation") {
        // only one primitive
        CHECK(state.bvh.primitives().size() == 1);

        // root + dummy node
        CHECK(state.bvh.nodes().size() == 2);

        // check one level
        CHECK(state.bvh.numLevels() == 1);

        // check one leaf
        CHECK(state.bvh.numLeaves() == 1);
    }

}

TEST_CASE("Debug AABB") {
    Features features = {
        .enableShading = true,
        .enableAccelStructure = true,
        .shadingModel = ShadingModel::Lambertian
    };
    Scene scene = loadScenePrebuilt(SceneType::Dragon, DATA_DIR);
    BVH bvh(scene, features);
    RenderState state = {
        .scene = scene,
        .features = features,
        .bvh = bvh,
        .sampler = {}
    };

    SECTION("Traverse") {
        bvh.traverse(0, 2, 0);
    }
}

// The below tests are not "good" unit tests. They don't actually test correctness.
// They simply exist for demonstrative purposes. As they interact with the interfaces
// (scene, bvh_interface, etc), they allow you to verify that you haven't broken
// our grading interface. They should compile without changes. If they do
// not compile, neither will our grading tests!
TEST_CASE("InterfaceTest")
{
    // Setup a RenderState object with some defaults
    Features features = {
        .enableShading = true,
        .enableAccelStructure = false, // BVH is not actually active r.n.
        .shadingModel = ShadingModel::Lambertian
    };
    Scene scene = loadScenePrebuilt(SceneType::CornellBox, DATA_DIR);
    BVH bvh(scene, features);
    RenderState state = { .scene = scene, .features = features, .bvh = bvh, .sampler = {} };

    SECTION("BVH generation")
    {
        // There's something in here?
        CHECK(!state.bvh.primitives().empty());
    }

    SECTION("BVH traversal")
    {
        Ray ray = { .origin = glm::vec3(0), .direction = glm::vec3(1) };
        HitInfo hitInfo;

        // Hit something?
        CHECK(state.bvh.intersect(state, ray, hitInfo));
        CHECK(ray.t != std::numeric_limits<float>::max());
    }

    SECTION("Hit shading")
    {
        Ray ray = { .origin = glm::vec3(0), .direction = glm::vec3(1) };
        HitInfo hitInfo;
        state.bvh.intersect(state, ray, hitInfo);

        // Shaded something?
        glm::vec3 Lo = computeShading(state, ray.direction, -ray.direction, glm::vec3(1), hitInfo);
        CHECK(glm::any(glm::notEqual(Lo, glm::vec3(0))));
    }
}
