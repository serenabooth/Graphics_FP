////////////////////////////////////////////////////////////////////////
//
//   Harvard Computer Science
//   CS 175: Computer Graphics
//   Professor Steven Gortler
// 
//
//   Modified by Serena Booth
//
////////////////////////////////////////////////////////////////////////
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#ifdef __MAC__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#   include <GL/glew.h>
#   include <GL/glut.h>
#endif

#include "ppm.h"
#include "glsupport.h"

using namespace std;      // for string, vector, iostream and other standard C++ stuff
using namespace std::tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// !!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!
// Before you start working on this assignment, set the following variable properly
// to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or OpenGL 3.x+ with
// GLSL 1.3.
//
// You should probably NOT turn on this flag.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get loaded
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
static const bool g_Gl2Compatible = false;


static int g_width             = 512;       // screen width
static int g_height            = 512;       // screen height
static bool g_leftClicked      = false;     // is the left mouse button down?
static bool g_rightClicked     = false;     // is the right mouse button down?
static float g_objScale        = 1.0;       // scale factor for object
static int g_leftClickX, g_leftClickY;      // coordinates for mouse left click event
static int g_rightClickX, g_rightClickY;    // coordinates for mouse right click event

static float x_dif = 0.0; 
static float y_dif = 0.0; 
// our global shader states
struct SquareShaderState {
  GlProgram program;

  // Handles to uniform variables
  GLint h_uVertexScaleWindowX; 
  GLint h_uVertexScaleWindowY; 

  GLint h_uVertexScale;
  GLint h_uTex0, h_uTex1;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aTexCoord;
};

/*
* Similar to Square
* Only one texture (h_uTex2)
* Break position into x and y coordinates (h_uVertexX, h_uVertexY)
*     to facilitate movement with ijkl keys. 
* Add h_aColor as addtl vertex attribute
*/
struct TriangleShaderState {
  GlProgram program;

  // Handles to uniform variables
  GLint h_uVertexScaleWindowX; 
  GLint h_uVertexScaleWindowY; 

  GLint h_uVertexX; 
  GLint h_uVertexY; 

  GLint h_uVertexScale;
  GLint h_uTex2;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aTexCoord;
  GLint h_aColor;
};

static shared_ptr<SquareShaderState> g_squareShaderState;
static shared_ptr<TriangleShaderState> g_triangleShaderState;

// our global texture instance
static shared_ptr<GlTexture> g_tex0;

// our global geometries
struct GeometryPX {
  GlArrayObject vao;
  GlBufferObject posVbo, texVbo, colorVbo;
};

static shared_ptr<GeometryPX> g_square;
static shared_ptr<GeometryPX> g_triangle;


// C A L L B A C K S ///////////////////////////////////////////////////
static void drawSquare() {
  // using a VAO is necessary to run on OS X.
  glBindVertexArray(g_square->vao);

  // activate the glsl program
  glUseProgram(g_squareShaderState->program);

  // bind textures
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, *g_tex0);

  // set glsl uniform variables
  safe_glUniform1i(g_squareShaderState->h_uTex0, 0); // 0 means GL_TEXTURE0
  safe_glUniform1f(g_squareShaderState->h_uVertexScale, g_objScale);

  // which dimension is more restrictive? resize according to that as if the window were square
  if (g_width < g_height) {
      safe_glUniform1f(g_squareShaderState->h_uVertexScaleWindowX, 1.0);
      safe_glUniform1f(g_squareShaderState->h_uVertexScaleWindowY, g_width * 1.0 / g_height);
  }
  else {
      safe_glUniform1f(g_squareShaderState->h_uVertexScaleWindowX,  g_height * 1.0 / g_width);
      safe_glUniform1f(g_squareShaderState->h_uVertexScaleWindowY, 1.0);
  }

  // bind vertex buffers
  glBindBuffer(GL_ARRAY_BUFFER, g_square->posVbo);
  safe_glVertexAttribPointer(g_squareShaderState->h_aPosition,
                             2, GL_FLOAT, GL_FALSE, 0, 0);

  glBindBuffer(GL_ARRAY_BUFFER, g_square->texVbo);
  safe_glVertexAttribPointer(g_squareShaderState->h_aTexCoord,
                             2, GL_FLOAT, GL_FALSE, 0, 0);

  safe_glEnableVertexAttribArray(g_squareShaderState->h_aPosition);
  safe_glEnableVertexAttribArray(g_squareShaderState->h_aTexCoord);

  // draw using 6 vertices, forming two triangles
  glDrawArrays(GL_TRIANGLES, 0, 6);

  safe_glDisableVertexAttribArray(g_squareShaderState->h_aPosition);
  safe_glDisableVertexAttribArray(g_squareShaderState->h_aTexCoord);

  glBindVertexArray(0);
  
  // check for errors
  checkGlErrors();
}

// _____________________________________________________
//|                                                     |
//|  display                                            |
//|_____________________________________________________|
///
///  Whenever OpenGL requires a screen refresh
///  it will call display() to draw the scene.
///  We specify that this is the correct function
///  to call with the glutDisplayFunc() function
///  during initialization

static void display(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawSquare();

  glutSwapBuffers();

  // check for errors
  checkGlErrors();
}


// _____________________________________________________
//|                                                     |
//|  reshape                                            |
//|_____________________________________________________|
///
///  Whenever a window is resized, a "resize" event is
///  generated and glut is told to call this reshape
///  callback function to handle it appropriately.

static void reshape(int w, int h) {
  g_width = w;
  g_height = h;
  glViewport(0, 0, w, h);
  glutPostRedisplay();
}


// _____________________________________________________
//|                                                     |
//|  mouse                                              |
//|_____________________________________________________|
///
///  Whenever a mouse button is clicked, a "mouse" event
///  is generated and this mouse callback function is
///  called to handle the user input.

static void mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) {
    if (state == GLUT_DOWN) {
      // right mouse button has been clicked
      g_leftClicked = true;
      g_leftClickX = x;
      g_leftClickY = g_height - y - 1;
    }
    else {
      // right mouse button has been released
      g_leftClicked = false;
    }
  }
  if (button == GLUT_RIGHT_BUTTON) {
    if (state == GLUT_DOWN) {
      // right mouse button has been clicked
      g_rightClicked = true;
      g_rightClickX = x;
      g_rightClickY = g_height - y - 1;
    }
    else {
      // right mouse button has been released
      g_rightClicked = false;
    }
  }
}


// _____________________________________________________
//|                                                     |
//|  motion                                             |
//|_____________________________________________________|
///
///  Whenever the mouse is moved while a button is pressed,
///  a "mouse move" event is triggered and this callback is
///  called to handle the event.

static void motion(int x, int y) {
  const int newx = x;
  const int newy = g_height - y - 1;
  if (g_leftClicked) {
    g_leftClickX = newx;
    g_leftClickY = newy;
  }
  if (g_rightClicked) {
    float deltax = (newx - g_rightClickX) * 0.02;
    //g_objScale += deltax;

    g_rightClickX = newx;
    g_rightClickY = newy;
  }
  glutPostRedisplay();
}

static void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "drag right mouse to change square size\n";
    break;
  case 'q':
    exit(0);
  case 's':
    glFinish();
    writePpmScreenshot(g_width, g_height, "out.ppm");
    break;
  case 'k':
    y_dif -= 0.01; 
    break;
  case 'i':
    y_dif += 0.01; 
    break;
  case 'j':
    x_dif -= 0.01; 
    break;
  case 'l':
    x_dif += 0.01; 
    break;
  }
  glutPostRedisplay();
}

// H E L P E R    F U N C T I O N S ////////////////////////////////////
static void initGlutState(int argc, char **argv) {
  glutInit(&argc,argv);
#ifdef __MAC__
  glutInitDisplayMode(GLUT_3_2_CORE_PROFILE|GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH); // core profile flag is required for GL 3.2 on Mac
#else
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
#endif
  glutInitWindowSize(g_width, g_height);      // create a window
  glutCreateWindow("CS 175: Hello World");    // title the window

  glutDisplayFunc(display);                   // display rendering callback
  glutReshapeFunc(reshape);                   // window reshape callback
  glutMotionFunc(motion);                     // mouse movement callback
  glutMouseFunc(mouse);                       // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(254/255,254/255,254/255,0);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void loadSquareShader(SquareShaderState& ss) {
  const GLuint h = ss.program; // short hand

  if (!g_Gl2Compatible) {
    readAndCompileShader(ss.program, "shaders/asst1-sq-gl3.vshader", "shaders/asst1-sq-gl3.fshader");
  }
  else {
    readAndCompileShader(ss.program, "shaders/asst1-sq-gl2.vshader", "shaders/asst1-sq-gl2.fshader");
  }

  // Retrieve handles to uniform variables
  ss.h_uVertexScaleWindowX = safe_glGetUniformLocation(h, "uVertexScaleWindowX");
  ss.h_uVertexScaleWindowY = safe_glGetUniformLocation(h, "uVertexScaleWindowY");

  ss.h_uVertexScale = safe_glGetUniformLocation(h, "uVertexScale");
  ss.h_uTex0 = safe_glGetUniformLocation(h, "uTex0");

  // Retrieve handles to vertex attributes
  ss.h_aPosition = safe_glGetAttribLocation(h, "aPosition");
  ss.h_aTexCoord = safe_glGetAttribLocation(h, "aTexCoord");

  if (!g_Gl2Compatible)
    glBindFragDataLocation(h, 0, "fragColor");
  checkGlErrors();
}

static void initShaders() {
  g_squareShaderState.reset(new SquareShaderState);
  loadSquareShader(*g_squareShaderState);
}

static void loadSquareGeometry(const GeometryPX& g) {
  GLfloat pos[12] = {
    -.5, -.5,
    .5,  .5,
    .5,  -.5,

    -.5, -.5,
    -.5, .5,
    .5,  .5
  };

  GLfloat tex[12] = {
    0, 0,
    1, 1,
    1, 0,

    0, 0,
    0, 1,
    1, 1
  };

  glBindBuffer(GL_ARRAY_BUFFER, g.posVbo);
  glBufferData(
    GL_ARRAY_BUFFER,
    12*sizeof(GLfloat),
    pos,
    GL_STATIC_DRAW);
  checkGlErrors();

  glBindBuffer(GL_ARRAY_BUFFER, g.texVbo);
  glBufferData(
    GL_ARRAY_BUFFER,
    12*sizeof(GLfloat),
    tex,
    GL_STATIC_DRAW);
  checkGlErrors();
}

static void initGeometry() {
  g_square.reset(new GeometryPX());
  loadSquareGeometry(*g_square);

}

static void loadTexture(GLuint texHandle, const char *ppmFilename) {
  int texWidth, texHeight;
  vector<PackedPixel> pixData;

  ppmRead(ppmFilename, texWidth, texHeight, pixData);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texHandle);
  glTexImage2D(GL_TEXTURE_2D, 0, g_Gl2Compatible ? GL_RGB : GL_SRGB, texWidth, texHeight,
               0, GL_RGB, GL_UNSIGNED_BYTE, &pixData[0]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  checkGlErrors();
}

static void initTextures() {
  g_tex0.reset(new GlTexture());

  loadTexture(*g_tex0, "smiley.ppm");
}



// M A I N /////////////////////////////////////////////////////////////

// _____________________________________________________
//|                                                     |
//|  main                                               |
//|_____________________________________________________|
///
///  The main entry-point for the HelloWorld example
///  application.



int main(int argc, char **argv) {
  try {
    initGlutState(argc,argv);

#ifndef __MAC__
    glewInit(); // load the OpenGL extensions
#endif

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
#ifndef __MAC__
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
#endif

    initGLState();
    initShaders();
    initGeometry();
    initTextures();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
