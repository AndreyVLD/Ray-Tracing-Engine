#include "intersect.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <limits>


bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p) {
    float area1 = glm::dot(glm::cross(v1 - v0, p - v0), n);
    float area2 = glm::dot(glm::cross(v2 - v1, p - v1), n);
    float area3 = glm::dot(glm::cross(v0 - v2, p - v2), n);
    return (area1 >= 0 && area2 >= 0 && area3 >= 0) || (area1 <= 0 && area2 <= 0 && area3 <= 0);
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    if (glm::dot(plane.normal, ray.direction) == 0 || glm::dot(ray.origin, plane.normal) == plane.D)
        return false;
    float t = (plane.D - glm::dot(plane.normal, ray.origin)) / glm::dot(plane.normal, ray.direction);
    if (t > 0) {
        if (t < ray.t) {
            ray.t = t;
            return true;
        }
    }
    return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
    plane.D = glm::dot(plane.normal, v0);
    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    Plane plane = trianglePlane(v0, v1, v2);
    float prevT = ray.t;
    if (intersectRayWithPlane(plane, ray)) {

        glm::vec3 p = ray.origin + ray.t * ray.direction;
        if (pointInTriangle(v0, v1, v2, plane.normal, p)) {
            return true;
        }
    }
    ray.t = prevT;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    float a = glm::dot(ray.direction, ray.direction);
    float b = 2 * glm::dot(ray.direction, ray.origin - sphere.center);
    float c = glm::dot(ray.origin - sphere.center, ray.origin - sphere.center) - sphere.radius * sphere.radius;
    float discriminant = b * b - 4 * a * c;
    float x1 = (-b + sqrt(discriminant)) / (2 * a);
    float x2 = (-b - sqrt(discriminant)) / (2 * a);
    if (discriminant < 0 || (x1 < 0 && x2 < 0))
        return false;
    if (x1 > 0 && x1 < ray.t)
        ray.t = x1;
    if (x2 > 0 && x2 < ray.t)
        ray.t = x2;
    return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    glm::vec3 t0 = (box.lower - ray.origin) / ray.direction;
    glm::vec3 t1 = (box.upper - ray.origin) / ray.direction;

    float tinx = std::min(t0.x, t1.x);
    float toutx = std::max(t0.x, t1.x);

    float tiny = std::min(t0.y, t1.y);
    float touty = std::max(t0.y, t1.y);

    float tinz = std::min(t0.z, t1.z);
    float toutz = std::max(t0.z, t1.z);

    float tin = std::max(tinx, std::max(tiny, tinz));
    float tout = std::min(toutx, std::min(touty, toutz));

    if (tin > tout || tout < 0)
        return false;
    if (tin > 0 && tin < ray.t) {
        ray.t = tin;
    }

    if (tout > 0 && tout < ray.t) {
        ray.t = tout;
    }

    return true;
}
