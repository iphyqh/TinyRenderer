#include <cmath> 
#include <vector>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
	bool steep = false;
	if (std::abs(x0-x1) < std::abs(y0-y1)) {
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}

	if (x0 > x1) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}

	// Optimize the repeated divisions in the for loop
	int dx = x1 - x0;
	int dy = y1 - y0;
	float slope = std::abs(dy/float(dx));
	float derror2 = std::abs(dy) * 2;
	float error = 0;

	/*
	A small modification to the author's code that subtract 0.5 to indicate the center of pixel;
	And this also makes the line starts nicer, otherwise it looks a bit flatter at the beginning (it takes 
	half amount of incrementation to advance y for the first time compared to later pixels)
	*/
	int y = y0 - 0.5; 

	for(int x = x0; x <= x1; x++) {
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
			y += (y1>y0?1:-1);
			error -= dx * 2;
		}
	}
}

int main(int argc, char** argv) {
 	if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        for (int j=0; j<3; j++) {
            Vec3f v0 = model->vert(face[j]);
            Vec3f v1 = model->vert(face[(j+1)%3]);
            int x0 = (v0.x+1.)*width/2.;
            int y0 = (v0.y+1.)*height/2.;
            int x1 = (v1.x+1.)*width/2.;
            int y1 = (v1.y+1.)*height/2.;
            line(x0, y0, x1, y1, image, white);
        }
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

