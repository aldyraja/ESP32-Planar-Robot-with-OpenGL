/*
NOTES:
- openGL only (Open Graphics Library)
- wsad input for y_cmd and x_cmd
- with controller
- with data monitoring & data logging
*/

/////////////////////////////////////////////////////////////
/* Template OpengGL sengaja dibuat untuk kuliah robotik 
*  di Departemen Teknik Elektro
*  Bagi yang ingin memodifikasi untuk keperluan yang lain,
*  dipersilahkan dengan menuliskan acknowledgement pada
*    Dr. Abdul Muis, MEng.
*    Autonomous Control Electronics (ACONICS) Research Group
*    http://www.ee.ui.ac.id/aconics
*////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <GL/glut.h> // Header File For The (openGL Utility Toolkit) Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library

#include <unistd.h> // Header file for sleeping.

#include <math.h>
#include <fcntl.h>			/* File control definitions */
// #include <errno.h>			/* Error number definitions */

#include <termios.h>		/* POSIX terminal control definitions */

#include "planar.c"

/* ascii code for the escape key */
#define ESCkey	27

// untuk gambar //
/* define color */  
GLfloat green1[4]   = {0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]    = {0.1, 0.1, 1.0, 1.0};
// GLfloat blue2[4]    = {0.2, 0.2, 1.0, 1.0};
// GLfloat blue3[4]    = {0.3, 0.3, 1.0, 1.0};
// GLfloat yellow1[4]  = {0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]  = {0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4]    = {0.8, 0.5, 0.6, 1.0};
GLfloat yellow5[4]  = {0.8, 0.8, 0.0, 1.0};
// GLfloat abu2[4]     = {0.5, 0.5, 0.5, 1.0};
// GLfloat gray1[4]    = {0.1, 0.1, 0.1, 1.0};
// GLfloat gray2[4]    = {0.2, 0.2, 0.2, 1.0};
// GLfloat gray3[4]    = {0.3, 0.3, 0.3, 1.0};
// GLfloat gray4[4]    = {0.4, 0.4, 0.4, 1.0};
// GLfloat gray5[4]    = {0.5, 0.5, 0.5, 1.0};
GLfloat gray6[4]    = {0.6, 0.6, 0.6, 1.0};
GLfloat gray7[4]    = {0.7, 0.7, 0.7, 1.0};
GLfloat gray8[4]    = {0.8, 0.8, 0.7, 1.0};

/* The number/handle of our GLUT window */
int window;

clock_t utime, utimer;

/* To draw a quadric model */
GLUquadricObj *obj;

// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.3

#define Link1 L1  // 0.3
#define Link2 L2  // 0.2

// end untuk gambar //


void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal

// membuat trajectory perubahan nilai x_cmd dan y_cmd
void trajectory_line(float t);
// hanya rumus forward kinematic (sudut -> koordinat)
void forward_kinematic(float *x, float *y, float q1, float q2);
// hitung ddx dan ddy
void hitung_PIDController(float *ddx, float *ddy, float ex, float ey);
// hitung inverse jacobian
void inverse_jacobian(float *ddq1_ref, float *ddq2_ref, float ddx, float ddy, float q1, float q2);

void control_robot();
void keyboard(unsigned char key, int i, int j);

////* NOT DEFAULT *////
float waktutraj = 2.0;

float dt = 0.017;  // sampling time

int linemode = 0;
float t = 0;

float *tetha1=&q1;
float *tetha2=&q2;

float *x=&objx;  // xres
float x_init = 0;
float x_cmd = 0.25;
float x_d = 0;  // selalu sama dengan x_cmd, berdasarkan grafik yang di plot

float *y=&objy;  // yres
float y_init = 0;
float y_cmd = 0;
float y_d = 0;

float ddx = 0;
float ddy = 0;

float ex = 0;
float ey = 0;
float ex_old = 0;
float ey_old = 0;
float int_ex = 0; 
float int_ey = 0;
float der_ex = 0;
float der_ey = 0;  

float ddq1_ref = 0;
float ddq2_ref = 0; 

//Tuning parameter
float Kp = 15.0;
float Ki = 0.1;
float Kd = 0.01;
float k = 10;

float v1 = 0.0;
float v2 = 0.0;
float dq1 = 0.0;
float dq2 = 0.0;
//* END NOT DEFAULT *//

// untuk gambar //
void drawOneLine(double x1, double y1, double x2, double y2) {
  glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();
}
   
void model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void model_box(GLfloat width, GLfloat depth, GLfloat height, GLfloat *color1, GLfloat *color2, GLfloat *color3, int color) {
   width=width/2.0;depth=depth/2.0;height=height/2.0;
   glBegin(GL_QUADS);
// top
    if (color==1) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth, height);
    glVertex3f( width,-depth, height);
    glVertex3f( width, depth, height);
    glVertex3f(-width, depth, height);
   glEnd();
   glBegin(GL_QUADS);
// bottom
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth,-height);
   glEnd();
   glBegin(GL_QUAD_STRIP);
// sides
    if (color==1) 
	    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glVertex3f(-width,-depth, height);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth, height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth, height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth, height);
    glVertex3f(-width, depth,-height);
    glVertex3f(-width,-depth, height);
   glEnd();
}

void disp_floor(void) {
  int i,j,flagc=1;

  glPushMatrix();
  
  GLfloat dx=4.5,dy=4.5;
  GLint amount=15;
  GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}

void lighting(void) {
	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void disp_robot(void) {
  glPushMatrix();
    // Gambar base
    model_box(0.3, 0.5, 0.05, gray8, gray7, gray6,1);
    glTranslatef(Xoffset, Yoffset, Zoffset/2);
    model_cylinder(obj, 0.1, 0.1, Zoffset, 2, blue1, yellow2);

    // Menuju joint-1
    glTranslatef(0, 0, Zoffset/2);
    glRotatef(*tetha1*RTD,0,0,1);

    glPushMatrix();
      // Gambar link1
      glRotatef(90,0,1,0);
      glTranslatef(0,0,Link1/2);
      model_cylinder(obj, 0.03, 0.03, Link1, 2, pink6, yellow2);
    glPopMatrix();

    // Menuju joint-2
    glTranslatef(Link1,0,0);
    glRotatef(*tetha2*RTD,0,0,1);

    glPushMatrix();
      // Gambar link1-1
      glRotatef(90,0,1,0);
      glTranslatef(0,0,Link2/2);
      model_cylinder(obj, 0.03, 0.03, Link2, 2, yellow5, yellow2);
    glPopMatrix();

    glPushMatrix();
      // Gambar link2
      glRotatef(90,0,1,0);
      glTranslatef(0,0,Link2/2);
      model_cylinder(obj, 0.03, 0.03, Link2, 2, yellow5, yellow2);
    glPopMatrix();
    
  glPopMatrix();
}
//end untuk gambar//


//display callback//
void display(void) {
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
  // glLoadIdentity();  // Reset View
  disp_floor();
  
  disp_robot();

  /* since window is double buffered, 
     Swap the front and back buffers (used in double buffering). */
  glutSwapBuffers() ;
}

//idle callback//
void Sim_main(void) {
	unsigned long Xr=0, Yr=0, Xg=0, Yg=0, Xb=0, Yb=0;  // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx, Ry, Gx, Gy, Bx, By;  // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
  static int count=0;
  glutSetWindow(window);
  animate(count);
  display();
  /*
  put a call to glutPostResdisplay into idle callback function,
  so when GLUT is idle the display callback will get called.
  */
  // Retrieve_serial();

  // tidak realtime
  usleep(100000);

////* NOT DEFAULT *////
  control_robot();
//* END NOT DEFAULT *//
}

void init(void) {
   obj = gluNewQuadric();

   /* Clear background to (Red, Green, Blue, Alpha) */
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST); // Enables Depth Testing
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(60.0, 2, 0.2, 8);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0.0, 0.0, 1.4,  0.0, 0.0, 0.0,  1.0, 0.0, 0.0);
	 lighting();
	 
   /* When the shading model is GL_FLAT only one colour per polygon is used, 
      whereas when the shading model is set to GL_SMOOTH the colour of 
      a polygon is interpolated among the colours of its vertices.  */
   glShadeModel(GL_SMOOTH) ; 

   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);

}

// Main Program
int main(int argc, char** argv) {
  utime = clock();

  // Initialize GLUT
    /* Initialize GLUT state - glut will take any command line arguments 
      see summary on OpenGL Summary */  
    glutInit (&argc, argv);
    
    // Berikut jika ingin menggunakan serial port
    //fd = open_port();
    //init_port(fd);

    /* Select type of Display mode:   
      Double buffer 
      RGBA color
      Alpha components supported 
      Depth buffer */  
    //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
    /* set a 400 (width) x 400 (height) window and its position */
    glutInitWindowSize(1080,720);	
    glutInitWindowPosition (50, 100);

    /* Open a window */  
    window = glutCreateWindow ("Aldy Raja - 2006522890");

    /* Initialize our window. */
    init() ;
    init_robot();

    /* Register the function to do all our OpenGL drawing. */
    glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

    /* Start Event Processing Engine */ 
    glutMainLoop () ;
    /*
    glutMainLoop only calls the display callback
    when a glut event triggers the display callback,
    such as resizing the window, uncovering the window,
    or calling glutPostRedisplay.
    */

   return 0 ;
}           

////* NOT DEFAULT (MOST IMPORTANT) *////
// update posisi
void trajectory_line(float t) {
  if(t < 0.017) {
    // === membuat trajectory perubahan nilai x_cmd dan y_cmd === //
    x_init = *x;
    y_init = *y;

	  linemode = (linemode+1) % 2;  // switch high/low

    if(linemode==0){
      x_cmd = *x + 0.05;
      //y_cmd = *y + 0.2;
    }else if(linemode==1){
      x_cmd = *x - 0.05;
      //y_cmd = *y - 0.2;
    }
    // ========================================================== //
  }

	//linear traj
	// x_d = (x_cmd-x_init)*t/waktutraj + x_init;
	// y_d = (y_cmd-y_init)*t/waktutraj + y_init;
	
	//step traj
	x_d = x_cmd;
	y_d = y_cmd;
}

// hanya rumus forward kinematic (sudut -> koordinat)
void forward_kinematic(float *x, float *y, float q1_temp, float q2_temp) {
  *x = L1*cos(q1_temp) + L2*cos(q1_temp+q2_temp);
  *y = L1*sin(q1_temp) + L2*sin(q1_temp+q2_temp);
}

//hitung ddx dan ddy
void hitung_PIDController(float *ddx, float *ddy, float ex, float ey) {
  //derivative = dx/dt
  der_ex = (ex - ex_old)/dt;
  der_ey = (ey - ey_old)/dt;
  
  //integral += xdt
  int_ex += ex*dt;
  int_ey += ey*dt;

  //output ddx and ddy value 
  *ddx = Kp*ex + Ki*int_ex + Kd*der_ex;
  *ddy = Kp*ey + Ki*int_ey + Kd*der_ey;
}

//hitung inverse jacobian
void inverse_jacobian(float *ddq1_ref, float *ddq2_ref, float ddx, float ddy, float q1, float q2) {

  // if(sin(q2) == 0) {
  //   // mencegah nilai singular
  //   // detJ = l1 * l2 * sin(q2)
  //   // https://sajidnisar.github.io/posts/python_2rp_jacobian#Jacobian-and-Singularity-Analysis-of-a-2-Link-Planar-Manipulator-using-Python
  // } else {
    //matriks jacobian
    double jcb[2][2] = {

          {   -(L1*sin(q1)+L2*sin(q1+q2)),     -L2*sin(q1+q2)  },
          {   L1*cos(q1)+L2*cos(q1+q2),        L2*cos(q1+q2)   }
      
      };

    //adjoin = {d -c, -b a}
    double jcb_adjoin[2][2] = {

          {   jcb[1][1],          (-1)*jcb[0][1]    },
          {   (-1)*jcb[1][0],     jcb[0][0]         }

      };
    
    //det = ad - bc
    double jcb_det = jcb[0][0]*jcb[1][1] - jcb[0][1]*jcb[1][0];

    //inverse = (1/det) * adjoin
    double jcb_inv[2][2] = {
          {   jcb_adjoin[0][0]/jcb_det,   jcb_adjoin[0][1]/jcb_det    },
          {   jcb_adjoin[1][0]/jcb_det,   jcb_adjoin[1][1]/jcb_det    }
      };

    // menghitung percepatan sudut referensi
    *ddq1_ref = jcb_inv[0][0]*ddx*dt + jcb_inv[0][1]*ddy*dt;
    *ddq2_ref = jcb_inv[1][0]*ddx*dt + jcb_inv[1][1]*ddy*dt;
  // }
}

void control_robot() {
	static int i=0;

	forward_kinematic(x, y, q1, q2);
	trajectory_line(waktutraj*(t/waktutraj - trunc(t/waktutraj)));

	ex = x_d - *x;
  ey = y_d - *y;

	hitung_PIDController(&ddx, &ddy, ex, ey);

	ex_old = ex;
	ey_old = ey;

	inverse_jacobian(&ddq1_ref, &ddq2_ref, ddx, ddy, q1, q2);

	v1 = k*ddq1_ref;
  v2 = k*ddq2_ref;
 
  // dq1=dq1+ddq1*dt;
  // dq2=dq2+ddq2*dt;
  dq1 = dq1 + (2.083*v1 - 2.71*dq1)*dt;  // 2.083 dan 2.71 dari parameter motor
  dq2 = dq2 + (2.083*v2 - 2.71*dq2)*dt;
  
	q1 = q1 + dq1*dt;
	q2 = q2 + dq2*dt;

	// if(i % 10 == 0) {
  //   printf("t:%f q1:%.2f q2:%.2f x:%.2f y:%.2f cmdx:%.2f cmdy:%.2f\n",
	// 						((float)(clock() - utime)/CLOCKS_PER_SEC), q1*RTD, q2*RTD, *x, *y, x_cmd, y_cmd);
  // }

	if(i % 10 == 0) printf("t:%.2f q1:%.2f q2:%.2f x:%.2f y:%.2f cmdx:%.2f cmdy:%.2f\n",
							t, q1*RTD, q2*RTD, *x, *y, x_cmd, y_cmd);

	i++;

  // looping dengan menambahkan nilai dt terhadap t di setiap iterasi
	t = t + dt;  // waktu dan sampling time tidaklah realtime

	FILE *fptr = fopen("dataLog.csv", "a");

  // exiting program 
  if (fptr == NULL) {
    printf("Error!");
    exit(1);
  }

	// fprintf(fptr, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ((float)(clock() - utime)/CLOCKS_PER_SEC),q1*RTD,q2*RTD,*x,*y,x_d,y_d,x_cmd,y_cmd);
  fprintf(fptr, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t,q1*RTD,q2*RTD,*x,*y,x_d,y_d,x_cmd,y_cmd);
  fclose(fptr);

}
//* END NOT DEFAULT (MOST IMPORTANT) *//

void keyboard(unsigned char key, int i, int j) {
	 switch(key){
      case ESCkey: exit(1); break;

      case '1': *tetha1+=5*DTR; break;
      case '2': *tetha2+=5*DTR; break;
      case '!': *tetha1-=5*DTR; break;
      case '@': *tetha2-=5*DTR; break;

      ////* NOT DEFAULT *////
      case 'w': x_cmd+=0.01; break;
      case 's': x_cmd-=0.01; break;
      case 'a': y_cmd+=0.01; break;
      case 'd': y_cmd-=0.01; break;
      //* END NOT DEFAULT *//
   }
}

// plot grafik via openGL, dari video pak Muis
// for(i = 0; i < graph_label.totx; i++) {
//   if(graph_label.showx[i]) {
//     glColor3fv(graph_color[color_index]);
//     glBegin(GL_LINE_STRIP);
//     datatemp = first;
//     j = 0;
//     while(datatemp -> next) {
//       glVertex2f(j, datatemp -> data.x[i]);
//       datatemp = (graph_result_t_pointer *) datatemp -> next;
//       j++;
//     }
//     glEnd();
//     color_index++;
//   }
// }