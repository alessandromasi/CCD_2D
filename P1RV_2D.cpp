#include <Windows.h>

#define _USE_MATH_DEFINES
#include <cmath>

#ifdef __APPLE__
#include <GLUT/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include <iostream>
#include <vector>
#include <cassert>


// This function will convert an angle to the equivalent rotation in the range [-pi,pi]
double simplifyAngle(double angle)
{
	while (angle >  M_PI) {
		angle -= 2 * M_PI;
	}
	while (angle <= - M_PI) {
		angle += 2 * M_PI;
	}
	return angle;
}
// This class is used internally by the CalcIK_2D_CCD function to represent a bone in
// world space.
class Bone_2D_CCD_World {
public :
	double x;        // x position in world space
	double y;        // y position in world space
	double angle;    // angle in world space
	double cosAngle; // sine of angle
	double sinAngle; // cosine of angle
};


// This class is used to supply the CalcIK_2D_CCD function with a bone's representation
// relative to its parent in the kinematic chain.
class Bone_2D_CCD {
public:
	double angle; // angle in parent space (rad)
	double l; // bone length
	double min; // rotation limits
	double max;

	// constructers
	Bone_2D_CCD(double _l, double _min, double _max) {
		angle = 0;
		l = _l;
		min = _min;
		max = _max;
	}
	Bone_2D_CCD(double _l, double _lim) : Bone_2D_CCD(_l, -_lim, _lim) {
	}
	Bone_2D_CCD(double _l) : Bone_2D_CCD(_l, M_PI) {
	}
};



class robotnR {
	private:

	double arrivalDist;
	std::vector<Bone_2D_CCD> bones;

	public:

	robotnR(std::vector<double> L, std::vector<double> Min, std::vector<double> Max, double _arrivalDist) {
		for (unsigned int i = 0; i < L.size(); i++) {
			Bone_2D_CCD bone(L[i],Min[i],Max[i]);
			bones.push_back(bone);
		}
		arrivalDist = _arrivalDist;
	}
	robotnR(std::vector<double> L, std::vector<double> Lim, double _arrivalDist) {
		for (unsigned int i = 0; i < L.size(); i++) {
			Bone_2D_CCD bone(L[i], Lim[i]);
			bones.push_back(bone);
		}
		arrivalDist = _arrivalDist;
	}
	robotnR(std::vector<double> L, double _arrivalDist) {
		for (unsigned int i = 0; i < L.size(); i++) {
			Bone_2D_CCD bone(L[i], M_PI);
			bones.push_back(bone);
		}
		arrivalDist = _arrivalDist;
	}

	// move the robot arm to target _x, _y in robot coordinates
	void move(float _x, float _y) {
		while (CalcIK_2D_CCD(_x, _y) == "processing") {
		}
	}

	// draw arm using direct kinematics
	void draw() {
		double x = 0;
		double y = 0;
		double angle = 0;
		glBegin(GL_LINES);
		for (unsigned int i = 0; i < bones.size(); i++) {
			if (i%2)
				glColor3f(0.0, 1.0, 0.0);
			else
				glColor3f(0.0, 0.0, 1.0);
			glVertex2f(x, y);
			angle += bones[i].angle;
			x += bones[i].l * cos(angle);
			y += bones[i].l * sin(angle);
			glVertex2f(x, y);
		}
		glEnd();
	}
	
	// Given a bone chain located at the origin, this function will perform a single cyclic
	// coordinate descent (CCD) iteration. This finds a solution of bone angles that places
	// the final bone in the given chain at a target position. The supplied bone angles are
	// used to prime the CCD iteration. If a valid solution does not exist, the angles will
	// move as close to the target as possible. The user should resupply the updated angles 
	// until a valid solution is found (or until an iteration limit is met).
	//  
	// returns: Success when a valid solution was found.
	//          Processing when still searching for a valid solution.
	//          Failure when it can get no closer to the target.
	std::string CalcIK_2D_CCD(double targetX, double targetY) {
		// Set an epsilon value to prevent division by small numbers.
		const double epsilon = 0.0001;

		// Set max arc length a bone can move the end effector an be considered no motion
		// so that we can detect a failure state.
		const double trivialArcLength = 0.00001;


		int numBones = bones.size();
		assert(numBones > 0);

		double arrivalDistSqr = arrivalDist * arrivalDist;

		//===
		// Generate the world space bone data.
		std::vector<Bone_2D_CCD_World> worldBones;

		// Start with the root bone.
		Bone_2D_CCD_World rootWorldBone;
		rootWorldBone.angle = bones[0].angle;
		rootWorldBone.x = bones[0].l * cos(rootWorldBone.angle);
		rootWorldBone.y = bones[0].l * sin(rootWorldBone.angle);
		
		rootWorldBone.cosAngle = cos(rootWorldBone.angle);
		rootWorldBone.sinAngle = sin(rootWorldBone.angle);
		worldBones.push_back(rootWorldBone);

		// Convert child bones to world space.
		for (int boneIdx = 1; boneIdx < numBones; boneIdx++)
		{
			Bone_2D_CCD_World prevWorldBone = worldBones[boneIdx - 1];
			Bone_2D_CCD curLocalBone = bones[boneIdx];

			Bone_2D_CCD_World newWorldBone;
			newWorldBone.angle = prevWorldBone.angle + curLocalBone.angle;
			newWorldBone.cosAngle = cos(newWorldBone.angle);
			newWorldBone.sinAngle = sin(newWorldBone.angle);
			newWorldBone.x = prevWorldBone.x + newWorldBone.cosAngle * curLocalBone.l;
			newWorldBone.y = prevWorldBone.y + newWorldBone.sinAngle * curLocalBone.l;
			worldBones.push_back(newWorldBone);
		}

		//===
		// Track the end effector position (the final bone)
		double endX = worldBones[numBones - 1].x;
		double endY = worldBones[numBones - 1].y;

		//===
		// Perform CCD on the bones by optimizing each bone in a loop 
		// from the final bone to the root bone
		bool modifiedBones = false;
		for (int boneIdx = numBones -1; boneIdx > 0; boneIdx--)
		{
			// Get the vector from the current bone to the end effector position.
			double curToEndX = endX - worldBones[boneIdx-1].x;
			double curToEndY = endY - worldBones[boneIdx-1].y;
			double curToEndMag = sqrt(curToEndX * curToEndX + curToEndY * curToEndY);

			// Get the vector from the current bone to the target position.
			double curToTargetX = targetX - worldBones[boneIdx-1].x;
			double curToTargetY = targetY - worldBones[boneIdx-1].y;
			double curToTargetMag = sqrt(curToTargetX * curToTargetX + curToTargetY * curToTargetY);

			// Get rotation to place the end effector on the line from the current
			// joint position to the target postion.
			double cosRotAng;
			double sinRotAng;
			double endTargetMag = curToEndMag * curToTargetMag;
			if (endTargetMag <= epsilon){
				cosRotAng = 1;
				sinRotAng = 0;
			}
			else {
				cosRotAng = (curToEndX * curToTargetX + curToEndY * curToTargetY) / endTargetMag;
				sinRotAng = (curToEndX * curToTargetY - curToEndY * curToTargetX) / endTargetMag;
			}

			// Clamp the cosine into range when computing the angle (might be out of range
			// due to floating point error).
			double rotAng = acos(max(-1, min(1, cosRotAng)));
			// correct angle value regarding its sine
			if (sinRotAng < 0.0)
				rotAng = -rotAng;
			// apply rotation limits
			double M = bones[boneIdx].max;
			double m = bones[boneIdx].min;
			double newAngle = simplifyAngle(bones[boneIdx].angle + rotAng);
			if (newAngle > M || newAngle < m )
				rotAng = simplifyAngle(newAngle - M) < simplifyAngle(newAngle - m) ? M-bones[boneIdx].angle : m-bones[boneIdx].angle;
			cosRotAng = cos(rotAng);
			sinRotAng = sin(rotAng);

			// Rotate the end effector position
			endX = worldBones[boneIdx-1].x + cosRotAng * curToEndX - sinRotAng * curToEndY;
			endY = worldBones[boneIdx-1].y + sinRotAng * curToEndX + cosRotAng * curToEndY;

			// Rotate the current bone in local space
			bones[boneIdx].angle = simplifyAngle(bones[boneIdx].angle + rotAng);

			// Check for termination
			double endToTargetX = (targetX - endX);
			double endToTargetY = (targetY - endY);

			if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= arrivalDistSqr) {
				// We found a valid solution.
				return "success";
			}

			// Track if the arc length that we moved the end effector was
			// a nontrivial distance.
			if (!modifiedBones && abs(rotAng) * curToEndMag > trivialArcLength) {
				modifiedBones = true;
			}
		}

		// We have to handle the first bone (index 0) separately 
		// because the algorithm needs value for the previous bone

		// Get the vector from the current bone to the end effector position.
		double curToEndX = endX;
		double curToEndY = endY;
		double curToEndMag = sqrt(curToEndX * curToEndX + curToEndY * curToEndY);

		// Get the vector from the current bone to the target position.
		double curToTargetX = targetX;
		double curToTargetY = targetY;
		double curToTargetMag = sqrt(curToTargetX * curToTargetX + curToTargetY * curToTargetY);

		// Get rotation to place the end effector on the line from the current
		// joint position to the target postion.
		double cosRotAng;
		double sinRotAng;
		double endTargetMag = curToEndMag * curToTargetMag;
		if (endTargetMag <= epsilon) {
			cosRotAng = 1;
			sinRotAng = 0;
		}
		else {
			cosRotAng = (curToEndX * curToTargetX + curToEndY * curToTargetY) / endTargetMag;
			sinRotAng = (curToEndX * curToTargetY - curToEndY * curToTargetX) / endTargetMag;
		}

		// Clamp the cosine into range when computing the angle (might be out of range
		// due to floating point error).
		double rotAng = acos(max(-1, min(1, cosRotAng)));
		// correct angle value regarding its sine
		if (sinRotAng < 0.0)
			rotAng = -rotAng;	
		// apply rotation limits
		double M = bones[0].max;
		double m = bones[0].min;
		double newAngle = simplifyAngle(bones[0].angle + rotAng);
		if (newAngle > M || newAngle < m)
			rotAng = simplifyAngle(newAngle - M) < simplifyAngle(newAngle - m) ? M - bones[0].angle : m - bones[0].angle;
		cosRotAng = cos(rotAng);
		sinRotAng = sin(rotAng);

		// Rotate the end effector position.
		endX = cosRotAng * curToEndX - sinRotAng * curToEndY;
		endY = sinRotAng * curToEndX + cosRotAng * curToEndY;

		// Rotate the current bone in local space (this value is output to the user)
		bones[0].angle = simplifyAngle(bones[0].angle + rotAng);

		// Check for termination
		double endToTargetX = (targetX - endX);
		double endToTargetY = (targetY - endY);

		if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= arrivalDistSqr) {
			// We found a valid solution.
			return "success";
		}

		// Track if the arc length that we moved the end effector was
		// a nontrivial distance.
		if (!modifiedBones && abs(rotAng) * curToEndMag > trivialArcLength) {
			modifiedBones = true;
		}

		// We failed to find a valid solution during this iteration.
		if (modifiedBones)
			return "processing";
		else
			return "failure";
	}
};

// vector of arm lengths
std::vector<double> L{ 0.5,0.4,0.2,0.3,0.05};
// vector of limits
double lim = 0.66 * M_PI;
std::vector<double> Lim{ lim,lim,lim,lim,lim };
// arm with rotation limits
robotnR robot(L,Lim,0.00005);
// // arm without rotation limits 
// robotnR robot(L, 0.00005);

// window size
int viewWidth = 600;
int viewHeight = 600;

// target position in pixel coordinates
int xOrigin = 3*viewWidth/4;
int yOrigin = 3*viewHeight/4;

// target position in openGL coordinates 
// conveniently also in robot coordinates
float xTarget;
float yTarget;


void drawPoint(float x, float y, float r, int segments, float R, float G, float B) {
	// draw point of radius r at x,y in openGL coordinates
	// color R,G,B
	// segments is the number of triangle that forms the point
	glBegin(GL_TRIANGLE_FAN);
	glColor3f(R, G, B);
	glVertex2f(x, y);
	for (int n = 0; n <= segments; ++n) {
		float const t = 2 * M_PI * (float)n / (float)segments;
		glVertex2f(x + sin(t) * r, y + cos(t) * r);
	}
	glEnd();
}

void drawTarget() {
	// draw target at cursor position
	int segments = 10;
	float offset = 0.03;
	float r = 0.004;
	xTarget = 2 * (float)xOrigin / (float)viewWidth - 1;
	yTarget = - 2 * (float)yOrigin / (float)viewHeight + 1;
	float x, y;
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			if ((i+j)%2==0) {
				x = xTarget + i * offset;
				y = yTarget + j * offset;
				drawPoint(x, y, r, segments, 1, 0, 0);
			}			
		}
	}
}
	
void drawOrigin() {
	// draw point at (0,0), the origin of the arm
	int segments = 10;
	float r = 0.005;
	float x = 0;
	float y = 0;
	drawPoint(x, y, r, segments, 1, 0, 0);
}

// Définition de la fonction d'affichage
GLvoid affichage() {
	// Effacement du frame buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// On passe en matice modelview
	glMatrixMode(GL_MODELVIEW);
	// on charge la matrice identite
	glLoadIdentity();
	drawOrigin();
	//drawWorkspace();
	drawTarget();
	robot.move(xTarget, yTarget);
	robot.draw();

	glutSwapBuffers();
}

GLvoid clavier(unsigned char touche, int x, int y) {
	switch (touche) {
	case 'q': // quitter
	case 27:
		exit(0);
		break;
	}

	// Demande a GLUT de reafficher la scene
	glutPostRedisplay();
}

// Fonction de gestion du deplacement de la souris
void deplacementSouris(int x, int y) {

	// On ne fait quelque chose que si l'utilisateur
	// a deja clique quelque part avec le bouton gauche
	if (xOrigin >= 0 || yOrigin >= 0) {
		xOrigin = x;
		yOrigin = y;
	}
	glutPostRedisplay();
}

// Fonction de gestion des clics souris
void clicSouris(int button, int state, int x, int y) {

	// On ne fait quelque chose que sur le bouton gauche de la souris
	if (button == GLUT_LEFT_BUTTON) {
		// si l'on a clique sur le bouton gauche
		// on garde les positions de la souris au moment du clic gauche
		if (state == GLUT_DOWN) {
			xOrigin = x;
			yOrigin = y;
		}
	}
	glutPostRedisplay();
}

// Fonction de redimensionnement de la fenetre
void redimensionner(int w, int h) {

	viewWidth = w;
	viewHeight = h;

	// On evite une division par 0
	// la fenetre ne peut avoir une largeur de 0
	if (h == 0)
		h = 1;

	// Calcul du ratio
	float ratio = (w * 1.0) / h;

	// On passe en mode "matrice de projection"
	glMatrixMode(GL_PROJECTION);

	// on charge la matrice identite
	glLoadIdentity();

	// on definit le viewport pour prendre toute la fenetre
	glViewport(0, 0, w, h);

	// on repasse en mode "matrice modelview"
	glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {

	// Initialisation de GLUT
	glutInit(&argc, argv);

	// Choix du mode d'affichage
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Position initiale de la fenêtre GLUT
	glutInitWindowPosition(200, 200);

	// Taille initiale de la fenêtre GLUT
	glutInitWindowSize(viewWidth, viewHeight);

	// Création de la fenêtre GLUT
	glutCreateWindow("2 arm robot - 2D");

	// Définition de la couleur d'effacement du framebuffer OpenGL
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Définition des fonctionsde callbacks
	glutDisplayFunc(affichage);
	glutKeyboardFunc(clavier);
	glutMouseFunc(clicSouris);
	glutMotionFunc(deplacementSouris);
	glutReshapeFunc(redimensionner);

	// Lancement de la boucle infinie GLUT
	glutMainLoop();
}
