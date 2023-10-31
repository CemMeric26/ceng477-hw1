#include <iostream>
#include <limits>
#include <cmath>
#include "parser.h"
#include "ppm.h"

typedef unsigned char RGB[3];


RGB background_color;
float pos_inf = std::numeric_limits<float>::infinity();
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

    Ray(const parser::Vec3f& origin, const parser::Vec3f& direction) : origin(origin), direction(direction) {}

    parser::Vec3f at(float t) const {

        parser::Vec3f formula = add_vec3f(origin, scalar_multiply_vec3f(direction, t)); // origin + t*direction
        return formula;
    }
};


HitData hit_triangle(Ray& ray, parser::Scene& scene, parser::Triangle& triangle, int unique_id, parser::Vec3f triangle_normal);
HitData hit_sphere(Ray& ray, parser::Scene& scene, parser::Sphere& sphere, int unique_id);
HitData hit_mesh(Ray& ray, parser::Scene& scene, parser::Mesh& mesh, int unique_id);
parser::Vec3i apply_shading_to_pixel(int recursion_depth, parser::Scene& scene, HitData& hit_data, Ray& ray);

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

 
parser::Vec3f add_vec3f(parser::Vec3f& v1, parser::Vec3f& v2)
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


parser::Vec3f triangle_unitnormal_calc(parser::Vec3f& v0, parser::Vec3f& v1, parser::Vec3f& v2)
{
    parser::Vec3f v0v1 = substract_vec3f(v1, v0);

    parser::Vec3f v0v2 = substract_vec3f(v2, v0);
    parser::Vec3f normal = cross_product_vec3f(v0v1, v0v2);
    normal = normalize_vec3f(normal);
    return normal;
}

std::vector<parser::Vec3f> tirangle_normals(parser::Scene& scene)
{
    std::vector<parser::Vec3f> normals;
    
    int size_of_triangles = scene.triangles.size();

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
    // right = cross_product(gaze, up)
    parser::Vec3f right = cross_product_vec3f(camera.gaze, camera.up);
    // imagePlaneCenter = camera.position + near_distance * gaze
    parser::Vec3f imagePlaneCenter = add_vec3f(camera.position, scalar_multiply_vec3f(camera.gaze, camera.near_distance));

    // pixelPosition = imagePlaneCenter + (i + 0.5) * pixelWidth * right - (j + 0.5) * pixelHeight * up
    float pixelWidth = (camera.near_plane.x - camera.near_plane.y) / camera.image_width;
    float pixelHeight = (camera.near_plane.w - camera.near_plane.z) / camera.image_height;
    /* parser::Vec3f pixelPosition = add_vec3f(
        add_vec3f(
            imagePlaneCenter,
            scalar_multiply_vec3f(right, (i + 0.5) * pixelWidth - (camera.image_width / 2.0) * pixelWidth)
        ),
        scalar_multiply_vec3f(camera.up, (j + 0.5) * pixelHeight - (camera.image_height / 2.0) * pixelHeight)
    ); */

    parser::Vec3f pixelPosition = add_vec3f(
    add_vec3f(
        imagePlaneCenter,
        scalar_multiply_vec3f(right, (camera.image_width / 2.0) * pixelWidth - (i + 0.5) * pixelWidth)
    ),
    scalar_multiply_vec3f(camera.up, (camera.image_height / 2.0) * pixelHeight - (j + 0.5) * pixelHeight)
);

    // ray_direction = normalize(pixelPosition - camera.position)
    parser::Vec3f ray_direction = normalize_vec3f(substract_vec3f(pixelPosition, camera.position));

    Ray ray(camera.position, ray_direction);
    return ray;
}

// we should calculate the closes hit of the ray with the scene
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

    return {true, SPHERE, unique_id, hit_t, ray.at(hit_t), {}, sphere.material_id};

}

HitData hit_mesh(Ray& ray, parser::Scene& scene, parser::Mesh& mesh, int mesh_index)
{
    parser::Triangle mesh_triangle;
    int mesh_size = mesh.faces.size();

    float hit_t = pos_inf;
    int unique_id=0;

    for(int i=0; i<mesh_size; i++)
    {
        mesh_triangle.material_id = mesh.material_id;
        mesh_triangle.indices = mesh.faces[i];
        parser::Vec3f triangle_normal = global_meshTriangles_normals[mesh_index][i];

        HitData hit_data = hit_triangle(ray, scene, mesh_triangle, i,triangle_normal);
        if(hit_data.has_intersection)
        {
            hit_t = min(hit_data.t, hit_t);
            if(hit_t == hit_data.t)
            {
                unique_id = i;
            }
        }
    }
    if(hit_t != pos_inf) // if there is a hit
    {
        return {true, MESH, unique_id, hit_t, ray.at(hit_t), {}, mesh.material_id};
    }

    HitData hit_data = {false, MESH, unique_id , pos_inf, {}, {}, mesh.material_id};
    return hit_data;
}

bool shadowray_obj_intersect(Ray& ray, parser::Scene& scene)
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
        
        HitData hit_data = hit_triangle(ray, scene, triangle, i, triangle_normal);
        
        if(hit_data.has_intersection)
        {
            return true;
        }
    }
    // for each sphere
    for(int j=0; j<size_of_spheres; j++)
    {
        parser::Sphere sphere = scene.spheres[j];
        HitData hit_data = hit_sphere(ray, scene, sphere, j);
        if(hit_data.has_intersection)
        {
            return true;
        }
    }
    // for each mesh
    for(int k=0; k<size_of_meshes; k++)
    {
        parser::Mesh mesh = scene.meshes[k];
        HitData hit_data = hit_mesh(ray, scene, mesh, k);
        if(hit_data.has_intersection)
        {
            return true;
        }
    }

    return false;
}

parser::Vec3i compute_pixel_color(Ray& ray, HitData& hit_data, parser::Scene& scene, int recursion_depth)
{
    if(recursion_depth > scene.max_recursion_depth)
    {
        return {0,0,0};
    }
    if(hit_data.has_intersection)
    {
        // find the color at the closest hit

        return apply_shading_to_pixel(recursion_depth, scene, hit_data, ray);
    }
    else if(recursion_depth == 0)
    {
        // no intersection for primary ray
        return scene.background_color;
    }
    else
    {
        // avoid repeated addition of the background color for reflected rays
        return {0,0,0};
    }

}

parser::Vec3i apply_shading_to_pixel(int recursion_depth, parser::Scene& scene, HitData& hit_data, Ray& ray)
{
    parser::Vec3f color = scene.materials[hit_data.material_id].ambient; // we started adding with ambient light

    // for each point light
    int size_of_point_lights = scene.point_lights.size();
    for(int i=0; i<size_of_point_lights; i++)
    {
        parser::PointLight point_light = scene.point_lights[i];
        parser::Vec3f light_position = point_light.position;
        parser::Vec3f light_intensity = point_light.intensity;

        // shadow ray is generated
        parser::Vec3f shadow_ray_direction = substract_vec3f(light_position, hit_data.hit_point);
        Ray shadow_ray(hit_data.hit_point, shadow_ray_direction);

        parser::Vec3f normalized_shadow_ray_direction = normalize_vec3f(shadow_ray_direction);

        // closest hit is calculated
        // IMPORTANTE RISOLTATE
        // insteead of using closest_hit define a another function beacuse we don't need to calculate the closest hit we just need any HIT
        bool shadow_ray_hit= shadowray_obj_intersect(shadow_ray, scene);

        // if there is no intersection
        if(!shadowray_obj_intersect(shadow_ray, scene))
        {
            // diffuse light is calculated
            parser::Vec3f diffuse_light = scalar_multiply_vec3f(light_intensity, max(0, dot_product_vec3f(hit_data.normal, normalized_shadow_ray_direction))); // formula is I * max(0, dot_product(n, l))
            // specular light is calculated
            // formula is I * max(0, dot_product(n, h))^p 
            parser::Vec3f specular_light = scalar_multiply_vec3f(light_intensity, pow(max(0, dot_product_vec3f(hit_data.normal, shadow_ray_direction)), scene.materials[hit_data.material_id].phong_exponent));

            // diffuse light is added to the color
            // color = color + diffuse_light => diffuse_light = diffuse_factor * 
            color = add_vec3f(color, scalar_multiply_vec3f(scene.materials[hit_data.material_id].diffuse, point_light.intensity)); // we need  float here
            // specular light is added to the color
            color = add_vec3f(color, scalar_multiply_vec3f(scene.materials[hit_data.material_id].specular, point_light.intensity)); // and here
        }
    }


}

void Main_raytrace_Computer(parser::Scene& scene)
{
    parser::Camera camera = scene.cameras[0];
    int width = camera.image_width;
    int height = camera.image_height;
    unsigned char* image = new unsigned char [width * height * 3];

    // for each pixel
    for(int i=0; i<width; i++)
    {
        for(int j=0; j<height; j++)
        {
            // ray is generated
            Ray ray = generate_ray(camera, i, j);
            // closest hit is calculated
            HitData hit_data = closest_hit(ray, scene);

            if(hit_data.has_intersection)
            {
                
                parser::Vec3f pixel_color = { 255,   0,   0 };  // Red ;
                image[(i + j * width) * 3 + 0] = pixel_color.x;
                image[(i + j * width) * 3 + 1] = pixel_color.y;
                image[(i + j * width) * 3 + 2] = pixel_color.z;
            }
            else
            {
                image[(i + j * width) * 3 + 0] = scene.background_color.x;
                image[(i + j * width) * 3 + 1] = scene.background_color.y;
                image[(i + j * width) * 3 + 2] = scene.background_color.z;
            }
        }
    }

    write_ppm("test.ppm", image, width, height);
      
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
