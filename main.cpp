#include <cmath> 
#include <vector>
#include <algorithm> 
#include <iostream>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0,   255, 0,   255);
const int width  = 800;
const int height = 800;
Model *model = NULL;

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color) {
	bool steep = false;
	if (std::abs(p0.x-p1.x) < std::abs(p0.y-p1.y)) {
		std::swap(p0.x, p0.y);
		std::swap(p1.x, p1.y);
		steep = true;
	}

	if (p0.x > p1.x) {
		std::swap(p0.x, p1.x);
		std::swap(p0.y, p1.y);
	}

	// Optimize the repeated divisions in the for loop
	int dx = p1.x - p0.x;
	int dy = p1.y - p0.y;
	// float slope = std::abs(dy/float(dx));
	float derror2 = std::abs(dy) * 2;
	float error = 0;

	/*
	A small modification to the author's code that subtract 0.5 to indicate the center of pixel;
	And this also makes the line starts nicer, otherwise it looks a bit flatter at the beginning (it takes 
	half amount of incrementation to advance y for the first time compared to later pixels)
	*/
	int y = p0.y - 0.5; 

	for (int x = p0.x; x <= p1.x; x++) {
		// Note that the if conditional can be taken outside the loop to optimize
		if (steep) { 
			image.set(y, x, color);
		}
		else {
			image.set(x, y, color);
		}
		// error += slope;
		error += derror2;
		if (error > dx) {
			y += (p1.y>p0.y?1:-1);
			error -= dx * 2;
		}
	}
}

// sort 2d-points vertically
bool compareY (Vec2i v0,Vec2i v1) { return (v0.y < v1.y); } 
void triangle_line(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
	Vec2i verticesArr[] = {t0, t1, t2};
  	std::vector<Vec2i> vertices (verticesArr, verticesArr+3);   
	std::sort(vertices.begin(), vertices.end(), compareY);
	for (int i = 0; i < 3; i++) {
		line(vertices[i], vertices[(i+1)%3], image, color);
	}

	// the slope here is dx/dy, because we are incrementing x instead of y (which differs from line()).
	float slope01 = std::abs((vertices[0].x-vertices[1].x)/float(vertices[0].y-vertices[1].y));
	float slope12 = std::abs((vertices[1].x-vertices[2].x)/float(vertices[1].y-vertices[2].y));
	float slope02 = std::abs((vertices[0].x-vertices[2].x)/float(vertices[0].y-vertices[2].y));

	/* 
	Scheme for line sweeping: starts from vertices[0] and ends at vertices[2] (loop splits at vertices[1] though), 
	stepsize along each line projects to the y-axis as 1 pixel.
	*/
	// initialization of endpoints for the vertical line
	Vec2f p0 = Vec2f(vertices[0].x, vertices[0].y);
	Vec2f p1 = Vec2f(vertices[0].x, vertices[0].y);

	for (int y = vertices[0].y; y <= vertices[1].y; y++) {
		p0.x = vertices[0].x + slope02*(vertices[0].x<vertices[2].x?1:-1) * (y - vertices[0].y);
		p1.x = vertices[0].x + slope01*(vertices[0].x<vertices[1].x?1:-1) * (y - vertices[0].y);
		line(Vec2i(p0.x,y), Vec2i(p1.x,y), image, color);
	}
	// one boundary line of the scanning changes from v0->v1 to v1->v2
	for (int y = vertices[1].y; y <= vertices[2].y; y++) {
		p0.x = vertices[0].x + slope02*(vertices[0].x<vertices[2].x?1:-1) * (y - vertices[0].y);
		p1.x = vertices[1].x + slope12*(vertices[1].x<vertices[2].x?1:-1) * (y - vertices[1].y);
		line(Vec2i(p0.x,y), Vec2i(p1.x,y), image, color);
	}
}

Vec3f barycentric(Vec2i *pts, Vec2i P) { 
    Vec3f u = Vec3f(pts[2].x-pts[0].x, pts[1].x-pts[0].x, pts[0].x-P.x)^Vec3f(pts[2].y-pts[0].y, pts[1].y-pts[0].y, pts[0].y-P.y);
    /* `pts` and `P` has integer value as coordinates
       so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z); 
} 

void triangle(Vec2i *pts, TGAImage &image, TGAColor color) { 
    Vec2i bboxmin(image.get_width()-1,  image.get_height()-1); 
    Vec2i bboxmax(0, 0); 
    Vec2i clamp(image.get_width()-1, image.get_height()-1); 
    for (int i=0; i<3; i++) { 
        bboxmin.x = std::max(0, std::min(bboxmin.x, pts[i].x));
		bboxmin.y = std::max(0, std::min(bboxmin.y, pts[i].y));

		bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
		bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
    } 

    Vec2i P; 
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) { 
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) { 
            Vec3f bc_screen  = barycentric(pts, P); 
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue; 
            image.set(P.x, P.y, color); 
        }
    } 
} 

int main(int argc, char** argv) {


    TGAImage image(width, height, TGAImage::RGB); 
	
	if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

	for (int i=0; i<model->nfaces(); i++) { 
		std::vector<int> face = model->face(i); 
		Vec2i screen_coords[3]; 
		for (int j=0; j<3; j++) { 
			Vec3f world_coords = model->vert(face[j]); 
			screen_coords[j] = Vec2i((world_coords.x+1.)*width/2., (world_coords.y+1.)*height/2.); 
		} 
		triangle(screen_coords, image, TGAColor(rand()%255, rand()%255, rand()%255, 255)); 
	}

	// define light_dir
	Vec3f light_dir(0,0,-1); 
	for (int i=0; i<model->nfaces(); i++) { 
		std::vector<int> face = model->face(i); 
		Vec2i screen_coords[3]; 
		Vec3f world_coords[3]; 
		for (int j=0; j<3; j++) { 
			Vec3f v = model->vert(face[j]); 
			screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.); 
			world_coords[j]  = v; 
		} 
		Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]); 
		n.normalize(); 
		float intensity = n*light_dir; 
		if (intensity>0) { 
			triangle(screen_coords, image, TGAColor(intensity*255, intensity*255, intensity*255, 255)); 
		} 
	}

    // Vec2i t0[3] = {Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80)};
    // Vec2i t1[3] = {Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180)};
    // Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};

    // triangle(t0, image, red);
    // triangle(t1, image, white);
    // triangle(t2, image, green);

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

