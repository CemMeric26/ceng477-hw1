#include <iostream>
#include <limits>
#include <cmath>
#include <algorithm>
#include <thread>  
#include "parser.h"
#include "ppm.h"

typedef unsigned char RGB[3];


RGB background_color;
float pos_inf = std::numeric_limits<float>::max();
struct IntersectionResult {
    bool has_intersection;
    parser::Vec3f hit_point;
};


enum Type_of_Object {
    TRIANGLE,
    SPHERE,
    MESH
};

parser::Vec3f scalar_multiply_vec3f(parser::Vec3f v, float scalar);
parser::Vec3f add_vec3f(parser::Vec3f v1, parser::Vec3f v2);

struct HitData {
    bool has_intersection;
    Type_of_Object type;
    int id;
    float t;
    parser::Vec3f hit_point;
    parser::Vec3f normal;
    int material_id;

};

class Ray {
public:
    parser::Vec3f origin;
    parser::Vec3f direction;
    int reflection_depth = 0;

    Ray(const parser::Vec3f& origin, const parser::Vec3f& direction) : origin(origin), direction(direction) {}

    parser::Vec3f at(float t) const {

        parser::Vec3f formula = add_vec3f(origin, scalar_multiply_vec3f(direction, t)); // origin + t*direction
        return formula;
    }
};


HitData hit_triangle(Ray& ray, parser::Scene& scene, parser::Triangle& triangle, int unique_id, parser::Vec3f triangle_normal);
HitData hit_sphere(Ray& ray, parser::Scene& scene, parser::Sphere& sphere, int unique_id);
HitData hit_mesh(Ray& ray, parser::Scene& scene, parser::Mesh& mesh, int unique_id);
parser::Vec3f apply_shading_to_pixel(parser::Scene& scene, HitData& hit_data, Ray& ray);

std::vector<parser::Vec3f> global_triangle_normals; // triangles normals array
std::vector<std::vector<parser::Vec3f>> global_meshTriangles_normals; // mesh triangles normals array


float min(float a, float b)
{
    if(a < b)
        return a;
    else
        return b;
}
float max(float a, float b)
{
    if(a > b)
        return a;
    else
        return b;
}

float determinant(float a, float d, float g,
                 float b, float e, float h,
                 float c, float f, float i) {
    return a * (e * i - h * f) 
         - b * (d * i - g * f) 
         + c * (d * h - g * e);
}


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

parser::Vec3f cross_product_vec3f(parser::Vec3f& v1, parser::Vec3f& v2)
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

parser::Vec3f vector_multiply (parser::Vec3f v1, parser::Vec3f v2)
{
    parser::Vec3f v;
    v.x = v1.x * v2.x;
    v.y = v1.y * v2.y;
    v.z = v1.z * v2.z;
    return v;
}
parser::Vec3f negate(parser::Vec3f& v)
{
    parser::Vec3f v_negated;
    v_negated.x = -v.x;
    v_negated.y = -v.y;
    v_negated.z = -v.z;
    return v_negated;
}

parser::Vec3f triangle_unitnormal_calc(parser::Vec3f& v0, parser::Vec3f& v1, parser::Vec3f& v2)
{
    parser::Vec3f v0v1 = substract_vec3f(v1, v0);

    parser::Vec3f v0v2 = substract_vec3f(v2, v0);
    parser::Vec3f normal = cross_product_vec3f(v0v1, v0v2);
    normal = normalize_vec3f(normal);
    return normal;
}

/* template<typename T>
T clamp(T value, T min, T max) {
    return std::max(min, std::min(max, value));
} */

parser::Vec3i clamp_color(const parser::Vec3f& color) {
    parser::Vec3i clamped_color;
    if(color.x > 255)
        clamped_color.x = 255;
    else
        clamped_color.x = round(color.x);

    if(color.y > 255)
        clamped_color.y = 255;
    else
        clamped_color.y = round(color.y);

    if(color.z > 255)
        clamped_color.z = 255;
    else
        clamped_color.z = round(color.z);

    return clamped_color;
}


std::vector<parser::Vec3f> tirangle_normals(parser::Scene& scene)
{
    std::vector<parser::Vec3f> normals;

    int size_of_triangles = scene.triangles.size();
    normals.reserve(size_of_triangles);

    for(int i=0; i < size_of_triangles;i++)
    {
        parser::Triangle triangle = scene.triangles[i];
        parser::Vec3f v0 = scene.vertex_data[triangle.indices.v0_id - 1];
        parser::Vec3f v1 = scene.vertex_data[triangle.indices.v1_id - 1];
        parser::Vec3f v2 = scene.vertex_data[triangle.indices.v2_id - 1];

        parser::Vec3f triangle_normal = triangle_unitnormal_calc(v0, v1, v2);
        normals.push_back(triangle_normal);
    }

    return normals;
}

std::vector<std::vector<parser::Vec3f>> all_meshTriangles_normals(parser::Scene& scene)
{
    std::vector<std::vector<parser::Vec3f>> all_meshTriangles_normals;
    std::vector<parser::Vec3f> meshTriangles_normals;
    int size_of_meshes = scene.meshes.size();

    for(int i=0; i < size_of_meshes;i++)
    {
        parser::Mesh mesh = scene.meshes[i];
        int size_of_mesh_triangles = mesh.faces.size();
        for(int j=0; j < size_of_mesh_triangles;j++)
        {
            parser::Face triangle = mesh.faces[j];
            parser::Vec3f v0 = scene.vertex_data[triangle.v0_id - 1];
            parser::Vec3f v1 = scene.vertex_data[triangle.v1_id - 1];
            parser::Vec3f v2 = scene.vertex_data[triangle.v2_id - 1];

            parser::Vec3f triangle_normal = triangle_unitnormal_calc(v0, v1, v2);
            meshTriangles_normals.push_back(triangle_normal);
        }
        all_meshTriangles_normals.push_back(meshTriangles_normals);
        meshTriangles_normals.clear();
    }

    return all_meshTriangles_normals;
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
        return false;
    else
        return true;
    
}

bool isBackFacing(const parser::Vec3f& camera_position, const parser::Vec3f& triangle_vertex, const parser::Vec3f& triangle_normal) {
    // Compute the vector from the vertex to the camera
    parser::Vec3f vertex_to_camera = substract_vec3f(triangle_vertex, camera_position);

    // Normalize the vectors
    vertex_to_camera = normalize_vec3f(vertex_to_camera);
    parser::Vec3f normalized_triangle_normal = normalize_vec3f(triangle_normal);

    // Compute the dot product between the vector and the triangle normal
    float dot = dot_product_vec3f(vertex_to_camera, normalized_triangle_normal);

    // If the dot product is greater than 0, then the triangle is facing away from the camera
    return dot > 0;
}


Ray generate_ray(parser::Camera& camera, int i, int j)
{
    parser::Vec3f m, q, u, v, s;
    float left_side = camera.near_plane.x;
    float right_side = camera.near_plane.y;
    float bottom_side = camera.near_plane.z;
    float top_side = camera.near_plane.w;

    float s_u = (i+0.5) * (right_side - left_side) / camera.image_width;
    float s_v = (j+0.5) * (top_side - bottom_side) / camera.image_height;

    m = add_vec3f(camera.position, scalar_multiply_vec3f(camera.gaze, camera.near_distance));

    u = cross_product_vec3f(camera.gaze, camera.up);
    u = normalize_vec3f(u);
    v = cross_product_vec3f(u, camera.gaze);

    q = add_vec3f(m, scalar_multiply_vec3f(u, left_side));
    q = add_vec3f(q, scalar_multiply_vec3f(v, top_side));

    s = add_vec3f(q, scalar_multiply_vec3f(u, s_u));
    s = add_vec3f(s, scalar_multiply_vec3f(v, -s_v)); // carefull here

    Ray ray(camera.position, normalize_vec3f(substract_vec3f(s, camera.position)));
    return ray;


}

HitData closest_hit(Ray& ray, parser::Scene& scene)
{
    // for each object in the scene
    int size_of_triangles = scene.triangles.size();
    int size_of_spheres = scene.spheres.size();
    int size_of_meshes = scene.meshes.size();

    // we need to find the closest hit of the ray with the scene
    float hit_t = pos_inf;
    HitData closest_hit_data = {false, TRIANGLE, 0 , pos_inf, {}, {}, 0};

    // for each triangle
    for(int i=0; i<size_of_triangles; i++)
    {
        parser::Triangle triangle = scene.triangles[i];
        parser::Vec3f triangle_normal = global_triangle_normals[i];
        
        HitData hit_data = hit_triangle(ray, scene, triangle, i, triangle_normal);
        
        if(hit_data.has_intersection)
        {
            hit_t = min(hit_data.t, hit_t);
           if(hit_t == hit_data.t)
            {
                closest_hit_data = hit_data;
            }
        }
    }
    // for each sphere
    for(int j=0; j<size_of_spheres; j++)
    {
        parser::Sphere sphere = scene.spheres[j];
        HitData hit_data = hit_sphere(ray, scene, sphere, j);
        if(hit_data.has_intersection)
        {
            hit_t = min(hit_data.t, hit_t);
            if(hit_t == hit_data.t)
            {
                closest_hit_data = hit_data;
            }
        }
    }
    // for each mesh
    for(int k=0; k<size_of_meshes; k++)
    {
        parser::Mesh mesh = scene.meshes[k];
        HitData hit_data = hit_mesh(ray, scene, mesh, k);
        if(hit_data.has_intersection)
        {
            hit_t = min(hit_data.t, hit_t);
            if(hit_t == hit_data.t)
            {
                closest_hit_data = hit_data;
            }
        }
    }

    return closest_hit_data;

} 

HitData hit_sphere(Ray& ray, parser::Scene& scene, parser::Sphere& sphere, int unique_id)
{
    parser::Vec3f hit_point;
    float hit_t = pos_inf;

    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];
    float radius = sphere.radius;

    // d is the ray direction
    parser::Vec3f d = ray.direction;

    // o-c is center_O in your code
    parser::Vec3f center_O = substract_vec3f(ray.origin, center); // o-c

    // Calculate dot(d, o-c)
    float dot_d_centerO = dot_product_vec3f(d, center_O); 

    // Calculate dot(o-c, o-c)
    float dot_centerO_centerO = dot_product_vec3f(center_O, center_O);

    // Calculate dot(d, d)
    float dot_d_d = dot_product_vec3f(d, d);

    // Calculate discriminant inside the square root
    float discriminant = dot_d_centerO * dot_d_centerO - dot_d_d * (dot_centerO_centerO - radius * radius); // (d.(o-c))^2 - (d.d)((o-c).(o-c) - r^2)

    // If discriminant is negative, the ray doesn't intersect the sphere
    if (discriminant < 0) {
        // Handle this case
        return {false, SPHERE, unique_id, pos_inf, {}, {}, sphere.material_id} ;  // or whatever is appropriate in your context
    }

    // Calculate the two possible values for t (t1 and t2)
    float t1 = (-dot_d_centerO - sqrt(discriminant)) / dot_d_d;
    float t2 = (-dot_d_centerO + sqrt(discriminant)) / dot_d_d;

    if((t1 > 0 && t2 > 0) || t1 == t2)
    {
        hit_t = min( min(t1,t2) ,hit_t); 
            
    }

    hit_point = ray.at(hit_t);
    parser::Vec3f sphere_normal = substract_vec3f(hit_point, center);
    sphere_normal = normalize_vec3f(sphere_normal);

    return {true, SPHERE, unique_id, hit_t, hit_point, sphere_normal, sphere.material_id};

}

HitData hit_triangle(Ray& ray, parser::Scene& scene, parser::Triangle& triangle, int unique_id, parser::Vec3f triangle_normal)
{
    parser::Vec3f hit_point;

    parser::Vec3f v0 = scene.vertex_data[triangle.indices.v0_id - 1];
    parser::Vec3f v1 = scene.vertex_data[triangle.indices.v1_id - 1];
    parser::Vec3f v2 = scene.vertex_data[triangle.indices.v2_id - 1];


    // Check if the triangle is backface of the camera
    if(!isBackFacing(ray.origin, v0, triangle_normal))
    {
        parser::Vec3f direction = ray.direction;
        // a = v0
        parser::Vec3f v0_v1 = substract_vec3f(v0, v1); // v0-1
        parser::Vec3f v0_v2 = substract_vec3f(v0, v2); // v0-2
        parser::Vec3f v0_O = substract_vec3f(v0, ray.origin); // v0-Origin

        // Calculate determinant
        float detA = determinant(v0_v1.x, v0_v2.x, direction.x,
                                    v0_v1.y, v0_v2.y, direction.y,
                                    v0_v1.z, v0_v2.z, direction.z);
        
        
        // determinant check
        if(detA == 0)
        {
            HitData hit_data = {false, TRIANGLE, unique_id , pos_inf, {}, {}, triangle.material_id};
            return hit_data;
        }
        // Calculate t
        float t = determinant(v0_v1.x, v0_v2.x, v0_O.x,
                                v0_v1.y, v0_v2.y, v0_O.y,
                                v0_v1.z, v0_v2.z, v0_O.z) / detA;

        if(t < 0)
        {
            HitData hit_data = {false, TRIANGLE, unique_id , pos_inf, {}, {}, triangle.material_id};
            return hit_data;
        }

        // Calculate beta
        float beta = determinant(v0_O.x, v0_v2.x, direction.x,
                                    v0_O.y, v0_v2.y, direction.y,
                                    v0_O.z, v0_v2.z, direction.z) / detA;
        
        // Calculate gamma
        float gamma = determinant(v0_v1.x, v0_O.x, direction.x,
                                    v0_v1.y, v0_O.y, direction.y,
                                    v0_v1.z, v0_O.z, direction.z) / detA;
        
        

        // Check if the intersection is inside the triangle
        if(beta < 0 || gamma < 0 || beta + gamma > 1) // not inside the triangle
        {
            HitData hit_data = {false, TRIANGLE, unique_id , pos_inf, {}, {}, triangle.material_id};
            return hit_data;
        }

        return {true, TRIANGLE, unique_id, t, ray.at(t), triangle_normal, triangle.material_id};
    }
    else
    {
        HitData hit_data = {false, TRIANGLE, unique_id , pos_inf, {}, {}, triangle.material_id};
        return hit_data;
    }
 
}

HitData hit_mesh(Ray& ray, parser::Scene& scene, parser::Mesh& mesh, int mesh_index)
{
    parser::Triangle mesh_triangle;
    int mesh_size = mesh.faces.size();
    HitData hit_data;
    HitData closest_hit;

    float hit_t = pos_inf;
    int unique_id=0;

    for(int i=0; i<mesh_size; i++)
    {
        mesh_triangle.material_id = mesh.material_id;
        mesh_triangle.indices = mesh.faces[i];
        parser::Vec3f triangle_normal = global_meshTriangles_normals[mesh_index][i];
        
        hit_data = hit_triangle(ray, scene, mesh_triangle, i,triangle_normal);

        if(hit_data.has_intersection && hit_t > 0  && hit_data.t < hit_t)
        {
            hit_t = hit_data.t;
            closest_hit = hit_data;
            unique_id = i;
        }
    }
    if(hit_t < pos_inf) // if there is a hit
    {
        closest_hit.type = MESH;  // only obj type is changed
        closest_hit.id = unique_id;
        return closest_hit;
    }

    hit_data = {false, MESH, unique_id , pos_inf, {}, {}, mesh.material_id};
    return hit_data;
}

bool shadowray_obj_intersect(Ray& ray, parser::Scene& scene, parser::Vec3f light_position )
{
    // for each object in the scene
    int size_of_triangles = scene.triangles.size();
    int size_of_spheres = scene.spheres.size();
    int size_of_meshes = scene.meshes.size();

    // for each triangle
    for(int i=0; i<size_of_triangles; i++)
    {
        parser::Triangle triangle = scene.triangles[i];
        parser::Vec3f triangle_normal = global_triangle_normals[i];

        // no need to implement backface culling for shadow rays
        
        HitData hit_data = hit_triangle(ray, scene, triangle, i, triangle_normal);

        // loook at if hit point behind the light source
        float origin_to_light = vector_magnitude(substract_vec3f(light_position, ray.origin)); // l-s
        float origin_to_intersect = vector_magnitude(substract_vec3f(hit_data.hit_point, ray.origin )); // o-s

        if(hit_data.has_intersection && hit_data.t >= 0 && origin_to_light > origin_to_intersect )
        {
            return true;
        }
        
    }
     // for each mesh
    for(int k=0; k<size_of_meshes; k++)
    {
        parser::Mesh mesh = scene.meshes[k];
        // no need to implement backface culling for shadow rays
        HitData hit_data = hit_mesh(ray, scene, mesh, k);

        // loook at if hit point behind the light source
        float origin_to_light = vector_magnitude(substract_vec3f(light_position, ray.origin)); // l-s
        float origin_to_intersect = vector_magnitude(substract_vec3f(hit_data.hit_point, ray.origin )); // obj-s

        if(hit_data.has_intersection && hit_data.t >= 0 && origin_to_light > origin_to_intersect)
        {
            return true;
        }
    }

    // for each sphere
    for(int j=0; j<size_of_spheres; j++)
    {
        parser::Sphere sphere = scene.spheres[j];
        HitData hit_data = hit_sphere(ray, scene, sphere, j);

        // loook at if hit point behind the light source
        float origin_to_light = vector_magnitude(substract_vec3f(light_position, ray.origin)); // l-s
        float origin_to_intersect = vector_magnitude(substract_vec3f(hit_data.hit_point, ray.origin )); // o-s

        if(hit_data.has_intersection && hit_data.t >= 0 && origin_to_light > origin_to_intersect)
        {
            return true;
        }
    }
   
    return false;
}

parser::Vec3f diffuse_light_calc(parser::Vec3f& normalized_shadow_ray_direction, parser::Vec3f& k_d, parser::Vec3f& hit_normal, float distance_sqr, parser::Vec3f& light_intensity)
{
    
    parser::Vec3f diffuse_light;
    parser::Vec3f light_irradiance = scalar_multiply_vec3f(light_intensity, (1/(distance_sqr)));
    float cos_theta = max(0, dot_product_vec3f(normalized_shadow_ray_direction, hit_normal));

    diffuse_light.x = k_d.x * light_irradiance.x * cos_theta;
    diffuse_light.y = k_d.y * light_irradiance.y * cos_theta;
    diffuse_light.z = k_d.z * light_irradiance.z * cos_theta;

    return diffuse_light;
}

parser::Vec3f specular_light_calc(parser::Vec3f& normalized_shadow_ray_direction, parser::Vec3f& k_s, parser::Vec3f& hit_normal, float distance_sqr, parser::Vec3f& light_intensity, parser::Vec3f& ray_direction, float phong_exponent)
{
    // h = (w_i+w_o)/|w_i+w_o|

    parser::Vec3f half_vector = normalize_vec3f(add_vec3f(normalized_shadow_ray_direction, normalize_vec3f(negate(ray_direction)))); // (w_i+w_o)/|w_i+w_o|
    // max(0,n.h)
    float cos_alfa = max(0, dot_product_vec3f(hit_normal, half_vector));
    // cos_theta/(r^2)
    parser::Vec3f irradiance = scalar_multiply_vec3f(light_intensity,(1/(distance_sqr))) ;
    // specular light is calculated
    // formula is I * max(0, dot_product(n, h))^p 
    parser::Vec3f specular_factor = scalar_multiply_vec3f(irradiance, pow(cos_alfa, phong_exponent));
    parser::Vec3f specular_light = vector_multiply(k_s, specular_factor);

    return specular_light;
}

parser::Vec3f reflection_vector_calc(parser::Vec3f& incoming_ray_direction, parser::Vec3f& hit_normal)
{
    // r = 2(n.w_i)n - w_i ?? which one the formula
    // r =  w_i - 2(n.w_i)n 
    // n, w_i are all unit vector should i normalize
    incoming_ray_direction = normalize_vec3f(incoming_ray_direction);
    parser::Vec3f reflection_vector = substract_vec3f(incoming_ray_direction, scalar_multiply_vec3f(hit_normal, 2*dot_product_vec3f(hit_normal, incoming_ray_direction)));
    return reflection_vector;
}

parser::Vec3f compute_pixel_color(Ray& ray, parser::Scene& scene)
{

    if(ray.reflection_depth > scene.max_recursion_depth)
    {
        return {0,0,0};
    }
    HitData hit_data = closest_hit(ray, scene);
    if(hit_data.has_intersection)
    {
        // find the color at the closest hit point
        return apply_shading_to_pixel(scene, hit_data, ray);
    }

    else if (ray.reflection_depth == 0)
    {
        // no intersection for the primary ray
        parser::Vec3f background_color;
        background_color.x = scene.background_color.x;
        background_color.y = scene.background_color.y;
        background_color.z = scene.background_color.z;
        return background_color;
    }
    else
    {
        // avoid repeated addition of the background color for reflected rays
        return {0,0,0};
    }

}

parser::Vec3f apply_shading_to_pixel(parser::Scene& scene, HitData& hit_data, Ray& ray)
{
    // changing to material_id-1
    parser::Vec3f color ; // we started adding with ambient light
    color.x = scene.ambient_light.x * scene.materials[hit_data.material_id-1].ambient.x;
    color.y = scene.ambient_light.y * scene.materials[hit_data.material_id-1].ambient.y;
    color.z = scene.ambient_light.z * scene.materials[hit_data.material_id-1].ambient.z;
    
    
    // if mirror object
    if(scene.materials[hit_data.material_id-1].is_mirror)
    {
        // r vector is calculated
        parser::Vec3f reflection_ray_direction = reflection_vector_calc(ray.direction, hit_data.normal);

        // should i do this ??
        reflection_ray_direction = normalize_vec3f(reflection_ray_direction);

        // reflection ray is generated
        Ray reflection_ray(add_vec3f( hit_data.hit_point, scalar_multiply_vec3f(hit_data.normal, scene.shadow_ray_epsilon)) , reflection_ray_direction);

        // HitData reflection_hit_data = closest_hit(reflection_ray, scene);

        reflection_ray.reflection_depth = ray.reflection_depth + 1;

        parser::Vec3f reflection_color = compute_pixel_color(reflection_ray, scene);
        color.x += reflection_color.x * scene.materials[hit_data.material_id-1].mirror.x;
        color.y += reflection_color.y * scene.materials[hit_data.material_id-1].mirror.y;
        color.z += reflection_color.z * scene.materials[hit_data.material_id-1].mirror.z;
    }
    

    // for each point light
    int size_of_point_lights = scene.point_lights.size();

    for(int i=0; i<size_of_point_lights; i++)
    {
        parser::PointLight point_light = scene.point_lights[i];
        parser::Vec3f light_position = point_light.position;
        parser::Vec3f light_intensity = point_light.intensity;

        // shadow ray is generated
        parser::Vec3f shadow_ray_direction = substract_vec3f(light_position, hit_data.hit_point); // light_position - hit_point
        
        parser::Vec3f normalized_shadow_ray_direction = normalize_vec3f(shadow_ray_direction);

        
        // parser::Vec3f epsilon_ray = scalar_multiply_vec3f(normalized_shadow_ray_direction, scene.shadow_ray_epsilon); // shadow_ray_direction * epsilon
        parser::Vec3f epsilon_ray = scalar_multiply_vec3f(hit_data.normal, scene.shadow_ray_epsilon); // shadow_ray_direction * epsilon

        Ray shadow_ray(add_vec3f(hit_data.hit_point, epsilon_ray), normalized_shadow_ray_direction); // hit_point + epsilon_ray =origin, normalized_shadow_ray_direction = direction


        bool shadow_ray_hit= shadowray_obj_intersect(shadow_ray, scene, light_position);

        // if there is no intersection
        // not or not not
        if(!shadow_ray_hit)
        {
            float distance_sqr = dot_product_vec3f(shadow_ray_direction,shadow_ray_direction); // distance = shadow_ray_direction * shadow_ray_direction
            // diffuse light is calculated 
            parser::Vec3f diffuse_light = diffuse_light_calc(normalized_shadow_ray_direction, scene.materials[hit_data.material_id-1].diffuse, hit_data.normal, distance_sqr, light_intensity);
            
            // specular light is calculated
            parser::Vec3f specular_light = specular_light_calc(normalized_shadow_ray_direction, scene.materials[hit_data.material_id-1].specular, hit_data.normal, distance_sqr, light_intensity, ray.direction, scene.materials[hit_data.material_id-1].phong_exponent);
            
            color.x += diffuse_light.x + specular_light.x;
            color.y += diffuse_light.y + specular_light.y;
            color.z += diffuse_light.z + specular_light.z; 

        }
    }

    return color;

}

parser::Vec3i compute_color(Ray& ray, parser::Scene& scene)
{

    parser::Vec3f float_pixel_color = compute_pixel_color(ray, scene);  // Red ;

    parser::Vec3i pixel_color = clamp_color(float_pixel_color); 
    return pixel_color;
}

void Child_raytrace_Computer(parser::Camera& camera, parser::Scene& scene, unsigned char* image ,int min_height, int max_height, const int width)
{
    for(int j=min_height; j < max_height; j++)
    {
        for(int i=0; i < width; i++)
        {
            Ray ray = generate_ray(camera, i, j);
            parser::Vec3i pixel_color = compute_color(ray, scene);
            int index = (j * width + i) * 3;
            image[index] = pixel_color.x;
            image[index + 1] = pixel_color.y;
            image[index + 2] = pixel_color.z;
        }
    }

}

void Main_raytrace_Computer(parser::Scene& scene)
{
    parser::Camera camera = scene.cameras[0];
    int width = camera.image_width;
    int height = camera.image_height;
    unsigned char* image = new unsigned char [width * height * 3];

    // we should use thread for comutating the image parallel
    // thread implementation
    const int number_of_cores = std::thread::hardware_concurrency();

    if(number_of_cores == 0 || height < number_of_cores)
    {
        Child_raytrace_Computer(camera, scene, image, 0, height, width);
       
    }
    else
    {
        std::thread* threads = new std::thread[number_of_cores];
        int height_increase = height / number_of_cores;

        for(int i=0; i < number_of_cores; i++)
        {
            int min_height = i * height_increase;
            int max_height = (i== number_of_cores-1) ? height : (i+1) * height_increase;
            threads[i] = std::thread(&Child_raytrace_Computer, std::ref(camera), std::ref(scene), image, min_height, max_height, width);
        
           
        }
        for(int i=0; i < number_of_cores; i++)
        {
            threads[i].join();
        }
        delete[] threads;
    }

    write_ppm("test.ppm", image, width, height);
    // image = nullptr;
      
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

    global_meshTriangles_normals = all_meshTriangles_normals(scene);
    global_triangle_normals = tirangle_normals(scene);


    Main_raytrace_Computer(scene);


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

    /* int width = 640, height = 480;
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
    write_ppm("test.ppm", image, width, height); */

}
