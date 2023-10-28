#include <iostream>
#include "parser.h"
#include "ppm.h"

typedef unsigned char RGB[3];


RGB background_color;

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
        parser::Vec3f cam_position = camera.position;
        parser::Vec3f cam_gaze = camera.gaze;




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
