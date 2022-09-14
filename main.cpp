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
const int depth = 255;

Vec3f light_dir = Vec3f(1,-1,1).normalize();
Vec3f eye(1,1,3);
Vec3f center(0,0,0);

float *zbuffer = new float[width*height];
Model *model = NULL;

Vec3f m2v(Matrix m) {
    return Vec3f(m[0][0]/m[3][0], m[1][0]/m[3][0], m[2][0]/m[3][0]);
}

Matrix v2m(Vec3f v) {
    Matrix m(4, 1);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.f;
    return m;
}

Matrix viewport(int x, int y, int w, int h) {
    Matrix m = Matrix::identity(4);
    m[0][3] = x+w/2.f;
    m[1][3] = y+h/2.f;
    m[2][3] = depth/2.f;

    m[0][0] = w/2.f;
    m[1][1] = h/2.f;
    m[2][2] = depth/2.f;
    return m;
}

Matrix lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye-center).normalize();
    Vec3f x = (up^z).normalize();
    Vec3f y = (z^x).normalize();
    Matrix res = Matrix::identity(4);
    for (int i=0; i<3; i++) {
        res[0][i] = x[i];
        res[1][i] = y[i];
        res[2][i] = z[i];
        res[i][3] = -center[i];
    }
    return res;
}

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
    Vec3f u = (s[0]^s[1]);
	// If u[2] is zero then AC and AB form an angle of 0 degree so that sin(0) = 0, which means triangle ABC is degenerate
    if (std::abs(u[2])>1e-2)
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
	// If triangle ABC is degenerate generate negative coordinates, it will be thrown away by the rasterizator
    return Vec3f(-1,1,1); 
} 

void triangle(Vec3f *pts, Vec2i *uv, float *zbuffer, TGAImage &image, float *intensity) { 
	// https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/perspective-correct-interpolation-vertex-attributes
	// The above post explains why we cannot use the old barycentric coordinate interpolation, despite that I tried in futile, the line sweeping approach
	// below should be the right way to do it.
	Vec3i t0 = pts[0];
	Vec3i t1 = pts[1];
	Vec3i t2 = pts[2];
	float it0 = intensity[0];
	float it1 = intensity[1];
	float it2 = intensity[2];
    if (t0.y==t1.y && t0.y==t2.y) return; // Get rid of degenerate triangles
    if (t0.y>t1.y) { std::swap(t0, t1); std::swap(it0, it1); }
    if (t0.y>t2.y) { std::swap(t0, t2); std::swap(it0, it2); }
    if (t1.y>t2.y) { std::swap(t1, t2); std::swap(it1, it2); }

    int total_height = t2.y-t0.y;
    for (int i=0; i<total_height; i++) {
        bool second_half = i>t1.y-t0.y || t1.y==t0.y;
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y;
        float alpha = (float)i/total_height;
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here

		// First Interpolation:
        Vec3i A   =               t0  + Vec3f(t2-t0)*alpha;
        Vec3i B   = second_half ? t1  + Vec3f(t2-t1)*beta : t0  + Vec3f(t1-t0)*beta;

        float itA =               it0 + (it2-it0)*alpha;
        float itB = second_half ? it1 + (it2-it1)*beta : it0 + (it1-it0)*beta;
        if (A.x>B.x) { std::swap(A, B); std::swap(itA, itB); }

        for (int j=A.x; j<=B.x; j++) {
            float phi = B.x==A.x ? 1. : (float)(j-A.x)/(float)(B.x-A.x);
        
			// Second Interpolation: 
		    Vec3i   P = Vec3f(A) + Vec3f(B-A)*phi;
            float itP =     itA +   (itB-itA)*phi;
            int idx = P.x+P.y*width;
            if (zbuffer[idx]<P.z) {
                zbuffer[idx] = P.z;
                // TGAColor color = model->diffuse(uvP);
                image.set(P.x, P.y, TGAColor(255, 255, 255).scale(itP));
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


    TGAImage image(width, height, TGAImage::RGB);

    for (int i=0; i<width*height; i++) {
        zbuffer[i] = std::numeric_limits<int>::min();
    }
    

	Matrix ModelView = lookat(eye, center, Vec3f(0,1,0));
	Matrix Projection = Matrix::identity(4);
	Matrix ViewPort   = viewport(width/8, height/8, width*3/4, height*3/4);
    Projection[3][2] = -1.f/(eye-center).norm();

	for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
		Vec3f tex[3];
		Vec3f world_coords[3];
		Vec3f screen_coords[3];
		// Intensity changed from the one at the center to intensities at three vertices
		float intensity[3];		
        for (int j=0; j<3; j++) {
			Vec3f v = model->vert(face[j]); 
		  	
			// Changed from orthogonal projection to perspective projection for a camera fixied on a position on z axis.
			screen_coords[j] =  m2v(ViewPort*Projection*ModelView*v2m(v));

			world_coords[j]  = v;

			intensity[j] = model->norm(i, j)*light_dir;
		}
		
		// Triangle Normal
		Vec3f n = ((world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0])); 
		n.normalize();

		// The following line is no longer useful since we now interpolate intensity across the triangle using gouraud shading
		// float intensity = n*light_dir;
		
		//if (intensity>0) {
			Vec2i uv[3];
			for (int k=0; k<3; k++) {
				uv[k] = model->uv(i, k);
			}
			triangle(screen_coords, uv, zbuffer, image, intensity);
		//}
    }
	image.flip_vertically();
    image.write_tga_file("output.tga");
	
	TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
	for (int i=0; i<width; i++) {
		for (int j=0; j<height; j++) {
			zbimage.set(i, j, TGAColor(zbuffer[i+j*width], 1));
		}
	}
	zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	zbimage.write_tga_file("zbuffer.tga");



    delete model;
	delete [] zbuffer;
    return 0;
}

