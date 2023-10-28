#include <iostream>
#include <limits>
#include "parser.h"
#include "ppm.h"

typedef unsigned char RGB[3];


RGB background_color;

class Ray {
public:
    parser::Vec3f origin;
    parser::Vec3f direction;

    Ray(const parser::Vec3f& origin, const parser::Vec3f& direction) : origin(origin), direction(direction) {}

    parser::Vec3f at(float t) const {

        parser::Vec3f formula = add_vec3f(origin, scalar_multiply_vec3f(direction, t)); // origin + t*direction
        return formula;
    }
};
// we need surface normal calculator for tirangles and spheres
parser::Vec3f substract_vec3f(parser::Vec3f v1, parser::Vec3f v2)
{
    parser::Vec3f v;
    v.x = v1.x - v2.x;
    v.y = v1.y - v2.y;
    v.z = v1.z - v2.z;
    return v;
}
parser::Vec3f scalar_multiply_vec3f(parser::Vec3f v1, float scalar)
{
    parser::Vec3f v;
    v.x = v1.x * scalar;
    v.y = v1.y * scalar;
    v.z = v1.z * scalar;
    return v;
}

 
parser::Vec3f add_vec3f(parser::Vec3f v1, parser::Vec3f v2)
{
    parser::Vec3f v;
    v.x = v1.x + v2.x;
    v.y = v1.y + v2.y;
    v.z = v1.z + v2.z;
    return v;
}
parser::Vec3f cross_product_vec3f(parser::Vec3f v1, parser::Vec3f v2)
{
    parser::Vec3f v;
    v.x = v1.y*v2.z - v1.z*v2.y;
    v.y = v1.z*v2.x - v1.x*v2.z;
    v.z = v1.x*v2.y - v1.y*v2.x;
    return v;
}

parser::Vec3f normalize_vec3f(parser::Vec3f v)
{
    float length = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    parser::Vec3f v_normalized;
    v_normalized.x = v.x / length;
    v_normalized.y = v.y / length;
    v_normalized.z = v.z / length;
    return v_normalized;
}

parser::Vec3f triangle_unitnormal_calc(parser::Vec3f v0, parser::Vec3f v1, parser::Vec3f v2)
{
    parser::Vec3f v0v1 = substract_vec3f(v1, v0);

    parser::Vec3f v0v2 = substract_vec3f(v2, v0);
    parser::Vec3f normal = cross_product_vec3f(v0v1, v0v2);
    normal = normalize_vec3f(normal);
    return normal;
}

float vector_magnitude(parser::Vec3f v)
{
    float magnitude = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    return magnitude;
}

float dot_product_vec3f(parser::Vec3f v1, parser::Vec3f v2)
{
    float dot_product = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
    return dot_product;
}


bool backface_of_camera(parser::Vec3f camera_position, parser::Vec3f camera_gaze, parser::Vec3f triangle_normal)
{
    parser::Vec3f camera_to_triangle = substract_vec3f(camera_position, triangle_normal);
    float magnitude_product = (vector_magnitude(camera_to_triangle)*vector_magnitude(triangle_normal));
    float cos_theta = dot_product_vec3f(camera_to_triangle, triangle_normal)/ magnitude_product;
    if(cos_theta > 0 && cos_theta < 1)
        return true;
    else
        return false;
    
}

/* Ray generate_ray(parser::Camera camera, int i, int j)
{
    // Calculate w, u, v
    parser::Vec3f w = scalar_multiply_vec3f(normalize_vec3f(camera.gaze), -1); // Inverse and normalize gaze direction
    parser::Vec3f u = normalize_vec3f(cross_product_vec3f(camera.up, w));
    parser::Vec3f v = cross_product_vec3f(w, u);

    // Extract l, r, t, b from near_plane
    float l = camera.near_plane.x;
    float r = camera.near_plane.y;
    float t = camera.near_plane.z;
    float b = camera.near_plane.w;

    // Calculate m
    parser::Vec3f m = add_vec3f(camera.position, scalar_multiply_vec3f(w, camera.near_distance));

    // Calculate q
    parser::Vec3f q = add_vec3f(
        add_vec3f(m, scalar_multiply_vec3f(u, l)),
        scalar_multiply_vec3f(v, t)
    );

    // Calculate s_u and s_v
    float s_u = (i + 0.5f) * (r - l) / camera.image_width;
    float s_v = (j + 0.5f) * (t - b) / camera.image_height;

    // Calculate s
    parser::Vec3f s = add_vec3f(
        add_vec3f(q, scalar_multiply_vec3f(u, s_u)),
        scalar_multiply_vec3f(v, -s_v)  // Note the subtraction here
    );

    // Calculate ray direction
    parser::Vec3f d = substract_vec3f(s, camera.position);
    d = normalize_vec3f(d);

    // Return the ray
    return Ray(camera.position, d);
}
 */
Ray generate_ray(parser::Camera camera, int i, int j)
{
    // right = cross_product(gaze, up)
    parser::Vec3f right = cross_product_vec3f(camera.gaze, camera.up);
    // imagePlaneCenter = camera.position + near_distance * gaze
    parser::Vec3f imagePlaneCenter = add_vec3f(camera.position, scalar_multiply_vec3f(camera.gaze, camera.near_distance));

    // pixelPosition = imagePlaneCenter + (i + 0.5) * pixelWidth * right - (j + 0.5) * pixelHeight * up
    float pixelWidth = (camera.near_plane.x - camera.near_plane.y) / camera.image_width;
    float pixelHeight = (camera.near_plane.w - camera.near_plane.z) / camera.image_height;
    parser::Vec3f pixelPosition = add_vec3f(
        add_vec3f(
            imagePlaneCenter,
            scalar_multiply_vec3f(right, (i + 0.5) * pixelWidth - (camera.image_width / 2.0) * pixelWidth)
        ),
        scalar_multiply_vec3f(camera.up, (j + 0.5) * pixelHeight - (camera.image_height / 2.0) * pixelHeight)
    );
    // ray_direction = normalize(pixelPosition - camera.position)
    parser::Vec3f ray_direction = normalize_vec3f(substract_vec3f(pixelPosition, camera.position));

    Ray ray(camera.position, ray_direction);
    return ray;
}

// we should calculate the closes hit of the ray with the scene
parser::Vec3f closest_hit(Ray ray, parser::Scene scene)
{
    double pos_inf = std::numeric_limits<double>::infinity();
    parser::Vec3f closest_hit_point;
    float closest_hit_t = pos_inf;
    // for each triangle
    for(int i=0; i<scene.triangles.size(); i++)
    {
        parser::Triangle triangle = scene.triangles[i];
        parser::Vec3f v0 = scene.vertex_data[triangle.indices.v0_id-1];
        parser::Vec3f v1 = scene.vertex_data[triangle.indices.v1_id-1];
        parser::Vec3f v2 = scene.vertex_data[triangle.indices.v2_id-1];
        parser::Vec3f triangle_normal = triangle_unitnormal_calc(v0, v1, v2);
        if(backface_of_camera(ray.origin, ray.direction, triangle_normal))
        {
            // calculate t
            float t = dot_product_vec3f(substract_vec3f(v0, ray.origin), triangle_normal) / dot_product_vec3f(ray.direction, triangle_normal);
            // calculate hit point
            parser::Vec3f hit_point = ray.at(t);
            // check if the hit point is inside the triangle
            parser::Vec3f v0v1 = substract_vec3f(v1, v0);
            parser::Vec3f v0v2 = substract_vec3f(v2, v0);
            parser::Vec3f v0p = substract_vec3f(hit_point, v0);
            float dot00 = dot_product_vec3f(v0v2, v0v2);
            float dot01 = dot_product_vec3f(v0v2, v0v1);
            float dot02 = dot_product_vec3f(v0v2, v0p);
            float dot11 = dot_product_vec3f(v0v1, v0v1);
            float dot12 = dot_product_vec3f(v0v1, v0p);
            float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            if(u >= 0 && v >= 0 && u + v <= 1)
            {
                if(t < closest_hit_t)
                {
                    closest_hit_t = t;
                    closest_hit_point = hit_point;
                }
            }
        }
    }
}

void Main_raytrace_Computer(parser::Scene scene)
{
    parser::Vec3i background_color = scene.background_color;
    float shadow_ray_epsilon = scene.shadow_ray_epsilon;
    int max_recursion_depth = scene.max_recursion_depth;
    std::vector<parser::Camera> cameras = scene.cameras;
    parser::Vec3f ambient_light = scene.ambient_light;
    std::vector<parser::PointLight> point_lights = scene.point_lights;
    std::vector<parser::Material> materials = scene.materials;
    std::vector<parser::Vec3f> vertex_data = scene.vertex_data;
    std::vector<parser::Mesh> meshes = scene.meshes;
    std::vector<parser::Triangle> triangles = scene.triangles;
    std::vector<parser::Sphere> spheres = scene.spheres;

    // for each camera
    for(int i=0; i<cameras.size(); i++)
    {
        // to calculate the image we need followings:
        // camera (cam_position, cam_gaze, up, nearplane, near_dist, image_w, image_h, image_name)
        // lights (ambient, point_lights)
        // materials (ambient, diffuse, specular, mirror, phong_exponent)
        // vertex_data (vertex)
        // meshes (material_id, faces)
        // triangles (material_id, indices)
        // spheres (material_id, center_vertex_id, radius)
        //  compute_color() and apply_Shading() functions


        parser::Camera camera = cameras[i];
        std::vector<parser::PointLight> point_light = scene.point_lights ;



    }

    
}

void Child_raytrace_Computer()
{
    ;
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    //
    // Normally, you would be running your ray tracing
    // code here to produce the desired image.

    const RGB BAR_COLOR[8] =
    {
        { 255, 255, 255 },  // 100% White
        { 255, 255,   0 },  // Yellow
        {   0, 255, 255 },  // Cyan
        {   0, 255,   0 },  // Green
        { 255,   0, 255 },  // Magenta
        { 255,   0,   0 },  // Red
        {   0,   0, 255 },  // Blue
        {   0,   0,   0 },  // Black
    };

    int width = 640, height = 480;
    int columnWidth = width / 8;

    unsigned char* image = new unsigned char [width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }

    //  You should render as many images as the number of cameras.
    write_ppm("test.ppm", image, width, height);

}
