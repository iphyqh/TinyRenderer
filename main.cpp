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
const int width  = 1024;
const int height = 1024;
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

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) { 
    Vec3f s[2];
	// The original for (int i = 2; i--; ) is quite clever but I changed it to a more common one
    for (int i = 1; i >= 0; i--) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
	// If u[2] is zero then AC and AB form an angle of 0 degree so that sin(0) = 0, which means triangle ABC is degenerate
    if (std::abs(u[2])>1e-2)
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
	// If triangle ABC is degenerate generate negative coordinates, it will be thrown away by the rasterizator
    return Vec3f(-1,1,1); 
} 

void triangle(Vec3f *pts, Vec3f *tex, float *zbuffer, TGAImage &image, TGAImage &texture, float intensity) { 
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width()-1, image.get_height()-1);
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::max(0.f,      std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
        }
    }

	TGAColor textureColor;
	Vec3f P;
	Vec2f Ptex(0,0);
	for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f bc_screen  = barycentric(pts[0], pts[1], pts[2], P);
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
            
			P.z = 0;
			Ptex.x = 0;
			Ptex.y = 0;
            for (int i=0; i<3; i++) {
				P.z += pts[i][2]*bc_screen[i];
				Ptex.x += tex[i][0] * (bc_screen[i]);
				Ptex.y += tex[i][1] * (bc_screen[i]);
			}

			if (zbuffer[int(P.x+P.y*width)]<P.z) {
                zbuffer[int(P.x+P.y*width)] = P.z;
				textureColor = texture.get(int(Ptex.x*1024), int(Ptex.y*1024));
                image.set(P.x, P.y, TGAColor(textureColor.r*intensity, textureColor.g*intensity, textureColor.b*intensity));
            }
        }
    } 
} 

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}

int main(int argc, char** argv) {
	if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    float *zbuffer = new float[width*height];
    for (int i=width*height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    TGAImage image(width, height, TGAImage::RGB);
	TGAImage texture(1024, 1024, TGAImage::RGB);
	texture.read_tga_file("obj/african_head_diffuse.tga");
	texture.flip_vertically();
	Vec3f light_dir(0,0,-1);
	int count = 0;

    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f pts[3];
		Vec3f tex[3];
		Vec3f world_coords[3];
        for (int j=0; j<3; j++) {
			Vec3f v = model->vert(face[j*3]); 
			Vec3f vt = model->vt(face[j*3+1]);
		   
			pts[j] = world2screen(v);
			tex[j] = vt;
			world_coords[j]  = v;
		}
		
		// Triangle Normal
		Vec3f n = cross((world_coords[2]-world_coords[0]),(world_coords[1]-world_coords[0])); 
		n.normalize(); 

		float intensity = n*light_dir;
		if (intensity>0) {
			std::cout << intensity << std::endl;
			triangle(pts, tex, zbuffer, image, texture, intensity);
		}
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

