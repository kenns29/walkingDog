#include <windows.h>
#include <gl/GL.h>
#include <gl/glu.h>
#include "glut.h"
#include <time.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "imathvec.h"

using namespace Imath;
using namespace std;

GLUquadricObj *quadratic;

typedef Vec3<float> Vec3f;
typedef Vec3<int> Vec3i;

enum{
	MENU_FLAT_SHADING = 1,
	MENU_SMOOTH_SHADING,
	MENU_NORMAL_ON,
	MENU_NORMAL_OFF,
	MENU_MESH_MESH,
	MENU_MESH_SOLID,
	MENU_CYLINDER_DENSITY_12,
	MENU_CYLINDER_DENSITY_18,
	MENU_CYLINDER_DENSITY_36,
	MENU_CYLINDER_DENSITY_72
};

int shadeModel = GL_SMOOTH;
bool isNormalOn = false;

/* simple sleeep function */
void sleep(unsigned int mseconds){
	clock_t upper_1 = mseconds + clock();
	while (upper_1 > clock());
}


class Angle{
public:
	Angle(){angle = 0.0;}
	Angle(double _angle):angle(_angle){}

	double DEG2RAD(){
		return(angle*M_PI/180.0);
	}
	void setAngle(double _angle){
		angle = _angle;
	}
	double getAngle(){return angle;}

	void moveAngle(double _angle){
		angle += _angle;
	}
	void moveAnglePositive(double _angle){
		angle += _angle;
		if(angle < 0)
			angle += 360;
		else if(angle >= 360)
			angle -= 360;
	}
	Angle& operator=(const Angle& rhs){
		if(this != &rhs){
			angle = rhs.angle;
		}
		return *this;
	}
private:
	double angle;
};

/* camera position */
/*class Eye{
public:
	Eye(){x=y=z=lookX=lookY=lookZ=upX=upY=upZ=0.0;}
	Eye(double _x, double _y, double _z, double _lookX, double _lookY, double _lookZ):x(_x),
		y(_y), z(_z), lookX(_lookX), lookY(_lookY), lookZ(_lookZ){
			projectRadius = sqrt(pow(x, 2) + pow(z, 2));
			xzAngle.setAngle(atan(z/x)*180.0/M_PI);
			yzAngle.setAngle(atan(y/projectRadius)*180.0/M_PI);
			radius = sqrt(pow(projectRadius, 2) + pow(y, 2));
			upX = 0.0;
			upY = 1.0;
			upZ = 0.0;
	}
	void rotateXZ(double _angle){
		xzAngle.moveAnglePositive(_angle);
		x = projectRadius* cos(xzAngle.DEG2RAD());
		z = projectRadius* sin(xzAngle.DEG2RAD());
	}
	void rotateYZ(double _angle){
		if (_angle == 0) return;
		if (_angle < 0){
			if (yzAngle.getAngle() > -20){
				yzAngle.moveAngle(_angle);
				y = radius * sin(yzAngle.DEG2RAD());
				projectRadius = abs(radius *cos(yzAngle.DEG2RAD()));
				x = projectRadius* cos(xzAngle.DEG2RAD());
				z = projectRadius* sin(xzAngle.DEG2RAD());
			} 
		}
		if (_angle > 0){
			if (yzAngle.getAngle() < 80){
				yzAngle.moveAngle(_angle);
				y = radius * sin(yzAngle.DEG2RAD());
				projectRadius = abs(radius *cos(yzAngle.DEG2RAD()));
				x = projectRadius* cos(xzAngle.DEG2RAD());
				z = projectRadius* sin(xzAngle.DEG2RAD());
			}
		}
	}
	
	void zoom(double s){
		radius += s;
		if (radius <= 0.2){
			radius = 0.2;
		}
		y = radius * sin(yzAngle.DEG2RAD());
		projectRadius = abs(radius * cos(yzAngle.DEG2RAD()));
		x = projectRadius* cos(xzAngle.DEG2RAD());
		z = projectRadius* sin(xzAngle.DEG2RAD());

	}
	void zoomIn(){
		zoom(-0.5);
	}
	void zoomOut(){
		zoom(0.5);
	}
double x, y, z, lookX, lookY, lookZ, upX, upY, upZ;
double projectRadius;
double radius;
Angle xzAngle;
Angle yzAngle;
};*/

class Eye{
public:
	Eye(){x=y=z=lookX=lookY=lookZ=upX=upY=upZ=0.0;}
	Eye(float _x, float _y, float _z, float _lookX, float _lookY, float _lookZ):x(_x),
		y(_y), z(_z), lookX(_lookX), lookY(_lookY), lookZ(_lookZ){
			lookVector = Vec3f(lookX, lookY, lookZ) - Vec3f(x, y, z);
			upX = 0.0;
			upY = 1.0;
			upZ = 0.0;
	}
	
	void moveOnZ(float d){
		z += d;
		lookZ = z + lookVector.z;
	}
	void moveOnX(float d){
		x += d;
		lookX = x + lookVector.x;
	}
	void moveOnY(float d){
		y += d;
		lookY = y + lookVector.y;
	}
	void moveForward(){
		moveOnZ(-0.5);
	}
	void moveBackward(){
		moveOnZ(0.5);
	}
	void moveLeft(){
		moveOnX(-0.5);
	}
	void moveRight(){
		moveOnX(0.5);
	}
	void moveDown(){
		moveOnY(-0.5);
	}
	void moveUp(){
		moveOnY(0.5);
	}
	float x, y, z, lookX, lookY, lookZ, upX, upY, upZ;
protected:
	Vec3f lookVector;
};

/* Light Class : sub class of Eye */
class Light: public Eye{
public:
	Light() : Eye(){}
	Light(float _x, float _y, float _z) : Eye(_x, _y, _z, 0, 0, 0){}
	Light(const Eye& e){
		setLight(e);
	}
	void setLight(const Eye& e){
		x = e.x + 7;
		y = e.y + 10;
		z = e.z;
	}

	static bool isFollowOn;
};

bool Light::isFollowOn = true;

/* material class */
class Material{
public:
	Material(){}
	
	Material(GLfloat amb0, GLfloat amb1, GLfloat amb2, GLfloat amb3,
		     GLfloat dif0, GLfloat dif1, GLfloat dif2, GLfloat dif3,
			 GLfloat spec0, GLfloat spec1, GLfloat spec2, GLfloat spec3,
			 GLfloat shin){

		setMaterial(amb0, amb1, amb2, amb3,
		             dif0, dif1, dif2, dif3,
					 spec0, spec1, spec2, spec3,
					 shin);
	}
	void setMaterial(GLfloat amb0, GLfloat amb1, GLfloat amb2, GLfloat amb3,
					 GLfloat dif0, GLfloat dif1, GLfloat dif2, GLfloat dif3,
					 GLfloat spec0, GLfloat spec1, GLfloat spec2, GLfloat spec3,
					 GLfloat shin){

		mat_ambient[0] = amb0; mat_ambient[1] = amb1; mat_ambient[2] = amb2; mat_ambient[3] = amb3;
		mat_diffuse[0] = dif0; mat_diffuse[1] = dif1; mat_diffuse[2] = dif2; mat_diffuse[3] = dif3;
		mat_specular[0] = spec0; mat_specular[1] = spec1; mat_specular[2] = spec2; mat_specular[3] = spec3;
		mat_shininess[0] = shin;
		mat_emission[0] = 1.0; mat_emission[1] = 0.3; mat_emission[2] = 0.3; mat_emission[3] = 0.0;
	}

	void putMaterial(){
		glMaterialfv(GL_FRONT,GL_AMBIENT,mat_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	}
	void putGlowingMaterial(){
		putMaterial();
		glMaterialfv(GL_FRONT, GL_EMISSION, mat_emission);
	}
	GLfloat mat_ambient[4];
	GLfloat mat_diffuse[4];
	GLfloat mat_specular[4];
	GLfloat mat_shininess[1];
	GLfloat mat_emission[4];


};

/* define chrome material */
Material mat_chrome(0.25, 0.25, 0.25, 1.0,

					0.4, 0.4, 0.4, 1.0,

					0.774597, 0.774597, 0.774597, 1.0,

					76.8);

/* define jade material */
Material mat_jade(	0.135, 0.2225, 0.1575, 0.95,

					0.54, 0.89, 0.63, 0.95,

					0.316228, 0.316228, 0.316228, 0.95,

					12.8);
/* define emerald material */
Material mat_emerald(0.0215, 0.1745, 0.0215, 0.55,

					0.07568, 0.61424, 0.07568, 0.55,

					0.633, 0.727811, 0.633, 0.55,

					76.8);
/* define ruby material */
Material mat_ruby(	0.1745, 0.01175, 0.01175, 0.55,

					0.61424, 0.04136, 0.04136, 0.55,

					0.727811, 0.626959, 0.626959, 0.55,

					76.8);

/* define pearl material */
Material mat_pearl( 0.25, 0.20725, 0.20725, 0.922,

					1.0, 0.829, 0.829, 0.922,

					0.296648, 0.296648, 0.296648, 0.922,

					11.264);

/* define polished silver material */
Material mat_polishedSilver(0.23125, 0.23125, 0.23125, 1.0,

					0.2775, 0.2775, 0.2775, 1.0,

					0.773911, 0.773911, 0.773911, 1.0,

					89.6);

/* define polished gold material */
Material mat_polishedGold(0.24725, 0.2245, 0.0645, 1.0,

					0.34615, 0.3143, 0.0903, 1.0,

					0.797357, 0.723991, 0.208006, 1.0,

					83.2);

/* simple sphere class, also as node in the tree */
class Sphere{
public:
	Sphere(){
		radius = 0.5;
		slice = 40;
		stack = 40;
		child = NULL;
		sibling = NULL;
	}
	Sphere(float _radius, float _slice, float _stack){
		radius = _radius;
		slice = _slice;
		stack = _stack;
		child = NULL;
		sibling = NULL;
	}
	Sphere(float _radius): radius(_radius){
		slice = 40;
		stack = 40;
		child = NULL;
		sibling = NULL;
	}
	
	void updateMatrix(){
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	}

	void draw(){
		gluSphere(quadratic, radius, slice, stack);
	}

	void setType(unsigned int t){
		if (t <= BODY)
			type = BODY;
		else if(t >= EYE){
			type = EYE;
		}
		else{
			type = t;
		}
	}

	bool isBody(){
		if (type == BODY)
			return true;
		else
			return false;
	}
	bool isHead(){
		if(type == HEAD)
			return true;
		else 
			return false;
	}
	bool isLeg(){
		if(type == LEG)
			return true;
		else
			return false;
	}
	bool isTail(){
		if(type == TAIL)
			return true;
		else
			return false;
	}
	bool isEar(){
		if(type == EAR)
			return true;
		else 
			return false;
	}
	bool isNose(){
		if(type == NOSE)
			return true;
		else 
			return false;
	}
	bool isEye(){
		if(type == EYE)
			return true;
		else
			return false;
	}
	GLfloat matrix[16];
	float radius;
	Sphere *child;
	Sphere *sibling;
	unsigned int type;

	static const unsigned int BODY = 0;
	static const unsigned int HEAD = 1;
	static const unsigned int LEG = 2;
	static const unsigned int TAIL = 3;
	static const unsigned int EAR = 4;
	static const unsigned int NOSE = 5;
	static const unsigned int EYE = 6;
private:
	float slice;
	float stack;
};

/* tree of spheres */
class Dog{
public:
	Dog(){
		speed = 0;
		angle.setAngle(0);
		legRotateIndex.setAngle(0);
		legRotateSpeed = 0.0;
		isMoving = false;
		center.setValue(0.0, 0.0, 0.0);
	}

	/* set up tree for the dog */
	void createDog(){
		root = new Sphere();
		root->setType(Sphere::BODY);
	
		glPushMatrix();
		glScalef(2, 1, 1);
		root->updateMatrix();
		glPopMatrix();

		head = new Sphere();
		head->setType(Sphere::HEAD);
		root -> child = head;
		
		for(int i=0; i<2; i++){
			ear[i] = new Sphere();
			ear[i]->setType(Sphere::EAR);
		}
		head->child = ear[0];
		ear[0]->sibling = ear[1];

		for(int i=0; i<2; i++){
			eye[i] = new Sphere();
			eye[i]->setType(Sphere::EYE);
		}
		ear[1]->sibling = eye[0];
		eye[0]->sibling = eye[1];

		nose = new Sphere();
		nose->setType(Sphere::NOSE);

		eye[1]->sibling = nose;

		for(int i=0; i<4; i++){
			leg[i] = new Sphere();
			leg[i]->setType(Sphere::LEG);
		}
		for(int i=0; i<3; i++){
			leg[i]->sibling = leg[i+1];
		}
		head->sibling= leg[0];		

		for(int i=0; i<4; i++){
			sleg[i] = new Sphere();
			sleg[i]->setType(Sphere::LEG);
			leg[i]->child = sleg[i];
		}
		for(int i=0; i<3; i++){
			sleg[i]->sibling = sleg[i+1];
		}
	
		for(int i=0; i<4; i++){
			tail[i] = new Sphere();
			tail[i]->setType(Sphere::TAIL);
		}
		leg[3]->sibling = tail[0];
		for(int i=0; i<3; i++){
			tail[i]->child = tail[i+1];
		}

	}

	/* update matrix in each node */
	void updateDog(){
		glPushMatrix();
		glLoadIdentity();
		
		/* body matrix(root) */
		glPushMatrix();
		/*glRotatef(angle.getAngle(), 0, 1, 0);
		glTranslatef(-movedDistance, 0, 0);*/
		printf("%1.1f, %1.1f, %1.1f\n", center.x, center.y, center.z);
		glTranslatef(center.x, center.y, center.z);
		glRotatef(angle.getAngle(), 0, 1, 0);
		glTranslatef(-speed, 0, 0);
		
		glScalef(2, 1, 1);
		root->updateMatrix();
		glPopMatrix();

		/* head matrix */
		glPushMatrix();
		glTranslatef(abs(cos(45*M_PI/180.0))*(-1+ root->radius/4.0), (1-root->radius/4.0)*abs(sin(45*M_PI/180.0)), 0);
		glScalef(0.5, 1, 1);
		head->updateMatrix();
		glPopMatrix();
		
		/* ear matrix */
		glPushMatrix();
		glRotatef(30, 1, 0, 0);
		glTranslatef(0, 0, -head->radius);
		glScalef(0.2, 0.2, 0.5);
		ear[0]->updateMatrix();
		glPopMatrix();
		
		glPushMatrix();
		glRotatef(180-30, 1, 0, 0);
		glTranslatef(0, 0, -head->radius);
		glScalef(0.2, 0.2, 0.5);
		ear[1]->updateMatrix();
		glPopMatrix();

		/* eye matrix */
		glPushMatrix();
		glRotatef(30, 0, 0, -1);
		glRotatef(30, 0, 1, 0);
		glTranslatef(-head->radius, 0, 0);
		glScalef(0.2, 0.2, 0.2);
		eye[0]->updateMatrix();
		glPopMatrix();

		glPushMatrix();
		glRotatef(30, 0, 0, -1);
		glRotatef(-30, 0, 1, 0);
		glTranslatef(-head->radius, 0, 0);
		glScalef(0.2, 0.2, 0.2);
		eye[1]->updateMatrix();
		glPopMatrix();

		/* nose matrix */
		glPushMatrix();
		glRotatef(-20, 0, 0, -1);
		glTranslatef(-head->radius, 0, 0);
		glScalef(0.3, 0.2, 0.2);
		nose->updateMatrix();
		glPopMatrix();

		/* leg matrix */
		glPushMatrix();
		glScalef(0.5, 1, 1);
		glRotatef(-30, 1, 0, 0);
		glRotatef(-30 - legRotateSpeed*sin(legRotateIndex.DEG2RAD()), 0, 0, 1);
		glTranslatef(0, -root->radius*2*0.7, 0);
		glScalef(0.2, 0.7, 0.2);
		leg[0]->updateMatrix();
		glPopMatrix();

		glPushMatrix();
		glScalef(0.5, 1, 1);
		glRotatef(30, 1, 0, 0);
		glRotatef(-30 + legRotateSpeed*sin(legRotateIndex.DEG2RAD()), 0, 0, 1);
		glTranslatef(0, -root->radius*2*0.7, 0);
		glScalef(0.2, 0.7, 0.2);
		leg[1]->updateMatrix();
		glPopMatrix();

		glPushMatrix();
		glScalef(0.5, 1, 1);
		glRotatef(30, 1, 0, 0);
		glRotatef(30 + legRotateSpeed*sin(legRotateIndex.DEG2RAD()), 0, 0, 1);
		glTranslatef(0, -root->radius*2*0.7, 0);
		glScalef(0.2, 0.7, 0.2);
		leg[2]->updateMatrix();
		glPopMatrix();

		glPushMatrix();
		glScalef(0.5, 1, 1);
		glRotatef(-30, 1, 0, 0);
		glRotatef(30 - legRotateSpeed*sin(legRotateIndex.DEG2RAD()), 0, 0, 1);
		glTranslatef(0, -root->radius*2*0.7, 0);
		glScalef(0.2,0.7, 0.2);
		leg[3]->updateMatrix();
		glPopMatrix();


		for(int i=0; i<4; i++){
			glPushMatrix();
			
			glTranslatef(0, -root->radius*2*0.8, 0);
			sleg[i]->updateMatrix();
			glPopMatrix();
		}

		/* tail matrix */
		glPushMatrix();
		glScalef(0.5, 1, 1);
		glTranslatef(root->radius*1.1*2, -root->radius*0.3, 0);
		glRotatef(-45, 0, 0, 1);
		glScalef(0.4, 0.2, 0.2);
		tail[0]->updateMatrix();
		glPopMatrix();

		glPushMatrix();
		glTranslatef(root->radius, 0, 0);
		glScalef(1/0.4, 1/0.2, 1/0.2);
		glRotatef(-20, 0, 0, 1);
		glScalef(0.4, 0.2, 0.2);
		glTranslatef(root->radius, 0, 0);
		tail[1]->updateMatrix();
		glPopMatrix();

		glPushMatrix();
		glTranslatef(root->radius, 0, 0);
		glScalef(1/0.4, 1/0.2, 1/0.2);
		glRotatef(-20, 0, 0, 1);
		glScalef(0.4, 0.2, 0.2);
		glTranslatef(root->radius, 0, 0);
		tail[2]->updateMatrix();
		glPopMatrix();

		glPopMatrix();
	}

	/* traverse through the tree of spheres and draw the dog */
	void traverse(Sphere *r){
		
		if(r == NULL) return;
		
		glPushMatrix();
		glMultMatrixf(r->matrix);
		
		glPushAttrib(GL_LIGHTING_BIT);
		/* set material for each component */
		if(r->isBody())
			mat_chrome.putMaterial();
		else if(r->isLeg())
			mat_pearl.putMaterial();
		else if(r->isHead())
			mat_pearl.putMaterial();
		else if(r->isTail())
			mat_polishedSilver.putMaterial();
		else if(r->isEar() || r->isEye() || r->isNose())
			mat_ruby.putGlowingMaterial();
		/* draw component */
		r->draw();
		glPopAttrib();
		if(r->child != NULL) 
			traverse(r->child);
		glPopMatrix();
		if(r->sibling != NULL) 
			traverse(r->sibling);
	}

	void drawDog(){
		traverse(root);
	}

	void rotate(float ang){
		angle.moveAnglePositive(ang);
	}
	Vec3f getPointFromMatrix(Vec3f p, GLfloat matrix[]){
		Vec3f t;
		t.x = p.x * matrix[0] + p.y * matrix[4] + p.z * matrix[8] + matrix[12];
		t.y = p.x * matrix[1] + p.y * matrix[5] + p.z * matrix[9] + matrix[13];
		t.z = p.x * matrix[2] + p.y * matrix[6] + p.z * matrix[10] + matrix[14];
		return t;
	}
	void moveDog(){
		center = getPointFromMatrix(Vec3f(0.0, 0.0, 0.0), root->matrix);
	}
	void speedUp(){
		speed += 0.01;
		legRotateSpeed += 2;
		if (legRotateSpeed > 20)
			legRotateSpeed = 20;
	}
	void slowDown(){
		speed -= 0.01;
		if(speed < 0)
			speed = 0;
		legRotateSpeed -= 2;
		if (legRotateSpeed < 0)
			legRotateSpeed = 0;
	}
	void reset(){
		//speed = 0;
		center.setValue(0.0, 0.0, 0.0);
		glPushMatrix();
		glLoadIdentity();
		root->updateMatrix();
		glPopMatrix();
		//legRotateSpeed = 0;
	}

	void changeLegRotateIndex(float spd){
		legRotateIndex.moveAnglePositive(spd);
	}

	void resetLegRotateIndex(){
		legRotateIndex.setAngle(0);
	}
	Sphere *root;
	Sphere *head;
	Sphere *leg[4];
	Sphere *sleg[4];
	Sphere *tail[4];
	Sphere *eye[2];
	Sphere *ear[2];
	Sphere *nose;

	float speed;
	Angle angle;

	float legRotateSpeed;
	Angle legRotateIndex;
	bool isMoving;
	Vec3f center;
};
/* simple class for triangle */
class Triangle{
public:
	Triangle(){}
	Triangle(Vec3f v0, Vec3f v1, Vec3f v2){
		setTriangle(v0, v1, v2);
	}
	void setTriangle(Vec3f v0, Vec3f v1, Vec3f v2){
		vertex[0] = v0;
		vertex[1] = v1;
		vertex[2] = v2;
		normal = calcNormal().normalize();
		normalBase = calcCentroid();
	}

	/* calculate normal vector based on three vertices */
	Vec3f calcNormal(){
		return (vertex[1]-vertex[0]).cross( (vertex[2] - vertex[0]) );
	}
	/* calculate the centroid of the triangle as the base for normal vector */
	Vec3f calcCentroid(){
		return (vertex[0] + vertex[1] + vertex[2])/3.0;
	}
	/* draw per-face normal vector for each triangle */
	void drawNormal(){

		Vec3f normalTop = normalBase + normal/3.0;
		glBegin(GL_LINES);
			glVertex3f(normalBase.x, normalBase.y, normalBase.z);
			glVertex3f(normalTop.x, normalTop.y, normalTop.z);
		glEnd();
	}
	/* vertexes */
	Vec3f vertex[3];
	/* per face normal vector */
	Vec3f normal;
	/* base of the normal vector*/
	Vec3f normalBase;
};

/* Mesh Class */
class Mesh{
public:
	Mesh(){
		isSolid = false;
	}

	/* function that creates the ground plane */
	void setMesh(float _per_width, float _per_length, int _row_length, int _num_of_rows){
		row_length = _row_length;
		per_width = _per_width; 
		per_length = _per_length;
		num_of_rows = _num_of_rows;

		length = (num_of_rows -1) * per_length;
		width = (row_length -1) * per_width;
		/* total number of vertexes */
		int total_vertex = row_length * num_of_rows;

		/* create the vertex array */
		for(int j=0; j< num_of_rows; j++){
			for(int i=0; i< row_length; i++){
				vertexArray.push_back(Vec3f(i*per_width, 0.0, j*per_length));
			}
		}

		/* create the index array */
		/* traverse order: (counter-clockwise)
		   0,5   4
		   *     *

		   *     *
		   1     2,3        */
		for(int j=0; j< num_of_rows-1; j++){
			for(int i=0; i< row_length -1; i++){
				/* first traingle in the square */
				indexArray.push_back(i + j*row_length);
				indexArray.push_back(i + (j+1)*row_length);
				indexArray.push_back(i + (j+1)*row_length +1);
				trigArray.push_back(Triangle(vertexArray[i + j*row_length],vertexArray[i + (j+1)*row_length],vertexArray[i + (j+1)*row_length +1]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
				/* second triangle in the square */
				indexArray.push_back(i + (j+1)*row_length +1);
				indexArray.push_back(i + j*row_length + 1);
				indexArray.push_back(i + j*row_length);
				trigArray.push_back(Triangle(vertexArray[i + (j+1)*row_length +1],vertexArray[i + j*row_length + 1], vertexArray[i + j*row_length]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
		}

	}
	/* iterate through the vertex array using the index array and draw the whole mesh */
	void draw(){
		if(!isSolid){
			glBegin(GL_LINES);
				for(unsigned int i=0; i< indexArray.size(); i+=3){
					Vec3f v[3];
					for(int j =0; j < 3; j++){
						v[j] = vertexArray[indexArray[i+j]];
					}
					glNormal3f(normalArray[i].x, normalArray[i].y, normalArray[i].z);
					glVertex3f(v[0].x, v[0].y, v[0].z); glVertex3f(v[1].x, v[1].y, v[1].z);
					glVertex3f(v[1].x, v[1].y, v[1].z); glVertex3f(v[2].x, v[2].y, v[2].z);
					glVertex3f(v[2].x, v[2].y, v[2].z); glVertex3f(v[0].x, v[0].y, v[0].z);
				}
			glEnd();
		}
		else{
			glBegin(GL_TRIANGLES);
				for(unsigned int i=0; i< indexArray.size(); i+=3){
					Vec3f v[3];
					for(int j =0; j < 3; j++){
						v[j] = vertexArray[indexArray[i+j]];
					}
					glNormal3f(normalArray[i].x, normalArray[i].y, normalArray[i].z);
					glVertex3f(v[0].x, v[0].y, v[0].z); 
					glVertex3f(v[1].x, v[1].y, v[1].z); 
					glVertex3f(v[2].x, v[2].y, v[2].z);
				}
			glEnd();
		}
	}

	void drawNormals(){
		for(unsigned int i=0; i<trigArray.size(); i++){
			trigArray[i].drawNormal();
		}
	}


/* number of vertex in each row */
int row_length;
/* number of rows in the mesh */
int num_of_rows;
/* width between each pair of consecutive vertex in a row */
float per_width;
/* length between each pair of consecutive vertex in a column */
float per_length;
/* length and width of the whole mesh*/
float length, width;

bool isSolid;

vector<Vec3f> vertexArray;
vector<int> indexArray;
vector<Vec3f> normalArray;
/* per face normal is stored in triangle class */
vector<Triangle> trigArray;
};

/* mesh class for cylinder : sub class of Mesh */
class CylinderMesh : public Mesh{
public:
	CylinderMesh(){
		isSolid = false; 
		theta = 30;
	}
	CylinderMesh(GLfloat _radius, GLfloat _height){
		setMesh(_radius, _height);
		isSolid = false;
		theta = 30;
	}

	void resetMesh(){
		vertexArray.clear();
		indexArray.clear();
		normalArray.clear();
		trigArray.clear();
		setMesh(radius, height);
	}

	/* the overriden setMesh method */
	void setMesh(GLfloat _radius, GLfloat _height){
		radius = _radius;
		height = _height;

		/* creates the vertex array */
			/* creates the base */
		vertexArray.push_back(Vec3f(0.0, 0.0, 0.0));
		for (int i=0; i<360; i+=theta){
			vertexArray.push_back(Vec3f(radius * cos(i*M_PI/180.0), radius * sin(i*M_PI/180.0), 0.0));
		}

		int topOriginIndex = vertexArray.size();
			/* creates the top */
		vertexArray.push_back(Vec3f(0.0, 0.0, height));
		for (int i=0; i<360; i+=theta){
			vertexArray.push_back(Vec3f(radius * cos(i*M_PI/180.0), radius * sin(i*M_PI/180.0), height));
		}

		/* creates the index array */
			/* traverse the base : (counter-clockwise) */
			/* 
			         * 5
				        * 2,4
				*         *
                0,3       1                       */
		for(int i=1; i<= 360/theta; i++){
			if(i< 360/theta){
				indexArray.push_back(0);
				indexArray.push_back(i);
				indexArray.push_back(i+1);
				trigArray.push_back(Triangle(vertexArray[0], vertexArray[i], vertexArray[i+1]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
			else{
				indexArray.push_back(0);
				indexArray.push_back(i);
				indexArray.push_back(1);
				trigArray.push_back(Triangle(vertexArray[0], vertexArray[i], vertexArray[1]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
		}
			/* traverse the side : (counter-clockwise)*/
		for(int i=1; i <= 360/theta; i++){
			if(i< 360/theta){
				indexArray.push_back(topOriginIndex + i);
				indexArray.push_back(i);
				indexArray.push_back(i+1);
				trigArray.push_back(Triangle(vertexArray[topOriginIndex + i], vertexArray[i], vertexArray[i+1]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}

				indexArray.push_back(i+1);
				indexArray.push_back(topOriginIndex + i + 1);
				indexArray.push_back(topOriginIndex + i);
				trigArray.push_back(Triangle(vertexArray[i+1], vertexArray[topOriginIndex + i +1], vertexArray[topOriginIndex + i]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
			else{
				indexArray.push_back(topOriginIndex + i);
				indexArray.push_back(i);
				indexArray.push_back(1);
				trigArray.push_back(Triangle(vertexArray[topOriginIndex + i], vertexArray[i], vertexArray[1]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}

				indexArray.push_back(1);
				indexArray.push_back(topOriginIndex + 1);
				indexArray.push_back(topOriginIndex + i);
				trigArray.push_back(Triangle(vertexArray[1], vertexArray[topOriginIndex + 1], vertexArray[topOriginIndex + i]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
		}
			/* traverse the top : counter-clockwise */
		for(int i=1; i<= 360/theta; i++){
			if(i==1){
				indexArray.push_back(topOriginIndex);
				indexArray.push_back(topOriginIndex + 360/theta);
				indexArray.push_back(topOriginIndex + 1);
				trigArray.push_back(Triangle(vertexArray[topOriginIndex], vertexArray[topOriginIndex + 360/theta], vertexArray[topOriginIndex + 1]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
			else{
				indexArray.push_back(topOriginIndex);
				indexArray.push_back(topOriginIndex + i - 1);
				indexArray.push_back(topOriginIndex + i);
				trigArray.push_back(Triangle(vertexArray[topOriginIndex], vertexArray[topOriginIndex + i - 1], vertexArray[topOriginIndex + i]));
				for(int k =0; k<3; k++){
					normalArray.push_back(trigArray[trigArray.size()-1].normal);
				}
			}
		}

	}

	GLfloat radius;
	GLfloat height;
	int theta;
};
/***************************
 * Define Global Variables *
 ***************************/
Eye eye(1, 2, 30, 0, 0, 0);
Light light(eye);
Dog dog;
Mesh groundPlane;
CylinderMesh cylinderMesh;

void init(){
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	glEnable(GL_DEPTH_TEST); 
	glEnable(GL_NORMALIZE); 
	// Set properties of the solid material
	mat_emerald.putMaterial();
	
	
}

void drawCoord(){
	glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0); glVertex3f(100.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 100.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 100.0);
	glEnd();
}

void animation(){
	dog.moveDog();
	dog.changeLegRotateIndex(10.0);
	
	glutPostRedisplay();
	sleep(20);
}
void display(){
	glClearColor(0.0, 0.0, 0.0, 0.0);

	glShadeModel(shadeModel);
	// Set the light source properties
	GLfloat lightIntensity[]={1.0, 1.0f, 1.0f, 1.0f};
	GLfloat light_position[]={light.x, light.y, light.z, 0.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightIntensity);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, 800.0/600.0, 7, 100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/*printf("vector: (%1.2f, %1.2f, %1.2f)\n", eye.x, eye.y, eye.z);
	printf("look: (%1.2f, %1.2f, %1.2f)\n", eye.lookX, eye.lookY,eye.lookZ);
	printf("yzAngle = %1.2f\nxzAngle = %1.2f\n", eye.yzAngle.getAngle(), eye.xzAngle.getAngle());
		printf("radius = %1.2f\nprojectRadius = %1.2f\n", eye.radius, eye.projectRadius);*/
	gluLookAt(eye.x, eye.y, eye.z, eye.lookX, eye.lookY, eye.lookZ, eye.upX, eye.upY, eye.upZ);

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glColor3f(0.0, 1.0, 0.0);
	
	
	glPushMatrix();
	dog.updateDog();
	dog.drawDog();
	glPopMatrix();
	
	glPushMatrix();
	glTranslatef(-groundPlane.width/2.0, -1.2, -groundPlane.length/2.0);
	groundPlane.draw();
	if(isNormalOn){
		glPushAttrib(GL_LIGHTING_BIT);
		mat_ruby.putMaterial();
		groundPlane.drawNormals();
		glPopAttrib();
	}
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-7, -1.2, 0);
	glRotatef(90, -1, 0, 0);
	glPushAttrib(GL_LIGHTING_BIT);
	mat_ruby.putMaterial();
	cylinderMesh.draw();
	if(isNormalOn){
		glPushAttrib(GL_LIGHTING_BIT);
		mat_jade.putMaterial();
		cylinderMesh.drawNormals();
		glPopAttrib();
	}
	glPopAttrib();
	glPopMatrix();

	//drawCoord();
	glutSwapBuffers();

}
/* special key function */
void specialKey(int key, int x, int y){
	switch(key){
		case GLUT_KEY_LEFT:
			//eye.rotateXZ(5);
			eye.moveLeft();
			if(Light::isFollowOn)
				light.moveLeft();
			break;
		case GLUT_KEY_RIGHT:
			//eye.rotateXZ(-5);
			eye.moveRight();
			if(Light::isFollowOn)
				light.moveRight();
			break;
		case GLUT_KEY_DOWN:
			//eye.zoomOut();
			eye.moveBackward();
			if(Light::isFollowOn)
				light.moveBackward();
			break;
		case GLUT_KEY_UP:
			//eye.zoomIn();
			eye.moveForward();
			if(Light::isFollowOn)
				light.moveForward();
			break;
		default:
			break;
	}
	glutPostRedisplay();
}
/* key board function */
void keyBoard(unsigned char key, int x, int y){
	switch(key){
		case 'a':
		case 'A':
			//eye.rotateYZ(5);
			eye.moveUp();
			if(Light::isFollowOn)
				light.moveUp();
			break;
		case 's':
		case 'S':
			//eye.rotateYZ(-5);
			eye.moveDown();
			if(Light::isFollowOn)
				light.moveDown();
			break;
		case 'k':
		case 'K':
			dog.rotate(5);
			break;
		case 'l':
		case 'L':
			dog.rotate(-5);
			break;
		case 'f':
		case 'F':
			dog.speedUp();
			break;
		case 'd':
		case 'D':
			dog.slowDown();
			break;
		case 'r':
		case 'R':
			dog.reset();
			break;
		case 'c':
		case 'C':
			if(Light::isFollowOn)
				Light::isFollowOn = false;
			else{
				Light::isFollowOn = true;
				light.setLight(eye);
			}
			break;
		default:
			break;
		
	}
	glutPostRedisplay();
}

/* menu */
void menu(int entry){
	switch(entry){
		case MENU_FLAT_SHADING:
			shadeModel = GL_FLAT;
			break;
		case MENU_SMOOTH_SHADING:
			shadeModel = GL_SMOOTH;
			break;
		case MENU_NORMAL_ON:
			isNormalOn = true;
			break;
		case MENU_NORMAL_OFF:
			isNormalOn = false;
			break;
		case MENU_MESH_MESH:
			groundPlane.isSolid = false;
			cylinderMesh.isSolid = false;
			break;
		case MENU_MESH_SOLID:
			groundPlane.isSolid = true;
			cylinderMesh.isSolid = true;
			break;
		case MENU_CYLINDER_DENSITY_12:
			cylinderMesh.theta = 30;
			cylinderMesh.resetMesh();
			break;
		case MENU_CYLINDER_DENSITY_18:
			cylinderMesh.theta = 20;
			cylinderMesh.resetMesh();
			break;
		case MENU_CYLINDER_DENSITY_36:
			cylinderMesh.theta = 10;
			cylinderMesh.resetMesh();
			break;
		case MENU_CYLINDER_DENSITY_72:
			cylinderMesh.theta = 5;
			cylinderMesh.resetMesh();
		default:
			break;
	}
	glutPostRedisplay();
}

void create_menu(){
	int shadingId = glutCreateMenu(menu);
	glutAddMenuEntry("Flat Shading", MENU_FLAT_SHADING);
	glutAddMenuEntry("Smooth Shading", MENU_SMOOTH_SHADING);
	int normalId = glutCreateMenu(menu);
	glutAddMenuEntry("On", MENU_NORMAL_ON);
	glutAddMenuEntry("Off", MENU_NORMAL_OFF);
	int meshId = glutCreateMenu(menu);
	glutAddMenuEntry("Mesh Mode", MENU_MESH_MESH);
	glutAddMenuEntry("Solid Mode", MENU_MESH_SOLID);
	int cylinderId = glutCreateMenu(menu);
	glutAddMenuEntry("12", MENU_CYLINDER_DENSITY_12);
	glutAddMenuEntry("18", MENU_CYLINDER_DENSITY_18);
	glutAddMenuEntry("36", MENU_CYLINDER_DENSITY_36);
	glutAddMenuEntry("72", MENU_CYLINDER_DENSITY_72);
	int menuId = glutCreateMenu(menu);
	glutAddSubMenu("Shading Option", shadingId);
	glutAddSubMenu("Normal Visualization", normalId);
	glutAddSubMenu("Mesh Display Mode", meshId);
	glutAddSubMenu("Change Cylinder Density", cylinderId);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
}
int main(int argc, char **argv)
{
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
		glutInitWindowSize(800,600);
		glutInitWindowPosition(100,100);
		glutCreateWindow("Walking Dog");
		quadratic = gluNewQuadric();
		gluQuadricNormals(quadratic, GLU_SMOOTH);
		/* create the tree */
		dog.createDog();
		/* creates the ground mesh */
		groundPlane.setMesh(0.3, 0.2, 180, 180);
		/* creates the cylinder mesh */
		cylinderMesh.setMesh(1.0, 3.0);
		create_menu();
		glutDisplayFunc(display);
		glutIdleFunc(animation);
		glutSpecialFunc(specialKey);
		glutKeyboardFunc(keyBoard);
		
		init();
		glViewport(0, 0, 800, 600);
		glutMainLoop();

		return 0;
}