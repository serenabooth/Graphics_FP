////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////
#include <queue>
#include <cstddef>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <map>
#include <fstream>
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

#include "lSystemParser.h"

#include "ppm.h"
#include "cvec.h"
#include "matrix4.h"
#include "rigtform.h"
#include "glsupport.h"
#include "geometrymaker.h"
#include "arcball.h"
#include "scenegraph.h"
#include "sgutils.h"

#include "asstcommon.h"
#include "drawer.h"
#include "picker.h"

#include "geometry.h"
#include "material.h"
#include "renderstates.h"

#include "mesh.h"
using namespace std;
using namespace tr1;

#define MAX(a, b) ((a > b) ? a : b) 
#define MIN(a, b) ((a < b) ? a : b) 

static void initScene(); 
static void initAnimation();
static void constructTree(shared_ptr<SgTransformNode> base, shared_ptr<Material> trunk_material, shared_ptr<Material> leaf_material);

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.5.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.5. Use GLSL 1.5 unless your system does not support it.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
static shared_ptr<SgRbtNode> leafTransformNode;

static vector< shared_ptr<SgTransformNode> > jointNodes;
static std::priority_queue<int> leavesToAdd;
static bool simLeaves = false; 

static bool animating = false; 
static int animate_level = 3; 

static shared_ptr<SgTransformNode> tree_base; 

const bool g_Gl2Compatible = false;

static shared_ptr<SimpleGeometryPNX> g_cylinderGeometry;
static Mesh g_cylinderMesh; 

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 20.0;   // half the ground length

enum SkyMode {WORLD_SKY=0, SKY_SKY=1};

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static bool g_spaceDown = false;         // space state, for middle mouse emulation
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static SkyMode g_activeCameraFrame = WORLD_SKY;

static bool g_displayArcball = true;
static double g_arcballScreenRadius = 100; // number of pixels
static double g_arcballScale = 1;

static bool g_pickingMode = false;

static bool g_playingAnimation = false;

static int num_iterations = 6; 
static std::string tree_lookup = "Lsystems/l0.txt"; 
static int selected_tree = 0; 

static double branch_thickness = 1.0; 
static double branch_length = 1.0; 

static int leaves_on = 0; 
static int tree_height = 0; 

// -------- Materials
static shared_ptr<Material> g_brownDiffuseMat, 
                            g_greenDiffuseMat,
                            g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat, 
                            g_barkMat, 
                            g_grassMat, 
                            g_leafMat, 
                            g_cylinderMat;

shared_ptr<Material> g_overridingMaterial;

typedef SgGeometryShapeNode MyShapeNode;


// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_leaf, g_cylinder, g_cylinderTEST;

// --------- Scene

Cvec3 g_light1(6.0, 3.0, -10.0), g_light2(2, 3.0, 5.0);  // define two lights positions in world space

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_light1Node, g_light2Node, g_treeNode, g_cylinderNode;

static shared_ptr<SgRbtNode> g_currentCameraNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode;

class Animator {
public:
  typedef vector<shared_ptr<SgRbtNode> > SgRbtNodes;
  typedef vector<RigTForm> KeyFrame;
  typedef list<KeyFrame> KeyFrames;
  typedef KeyFrames::iterator KeyFrameIter;

private:
  SgRbtNodes nodes_;
  KeyFrames keyFrames_;

public:
  void attachSceneGraph(shared_ptr<SgNode> root) {
    nodes_.clear();
    keyFrames_.clear();
    dumpSgRbtNodes(root, nodes_);
  }

  void loadAnimation(const char *filename) {
    ifstream f(filename, ios::binary);
    if (!f)
      throw runtime_error(string("Cannot load ") + filename);
    int numFrames, numRbtsPerFrame;
    f >> numFrames >> numRbtsPerFrame;
    if (numRbtsPerFrame != nodes_.size()) {
      cerr << "Number of Rbt per frame in " << filename
           <<" does not match number of SgRbtNodes in the current scene graph.";
      return;
    }

    Cvec3 t;
    Quat r;
    keyFrames_.clear();
    for (int i = 0; i < numFrames; ++i) {
      keyFrames_.push_back(KeyFrame());
      keyFrames_.back().reserve(numRbtsPerFrame);
      for (int j = 0; j < numRbtsPerFrame; ++j) {
        f >> t[0] >> t[1] >> t[2] >> r[0] >> r[1] >> r[2] >> r[3];
        keyFrames_.back().push_back(RigTForm(t, r));
      }
    }
  }

  void saveAnimation(const char *filename) {
    ofstream f(filename, ios::binary);
    int numRbtsPerFrame = nodes_.size();
    f << getNumKeyFrames() << ' ' << numRbtsPerFrame << '\n';
    for (KeyFrames::const_iterator frameIter = keyFrames_.begin(), e = keyFrames_.end(); frameIter != e; ++frameIter) {
      for (int j = 0; j < numRbtsPerFrame; ++j) {
        const RigTForm& rbt = (*frameIter)[j];
        const Cvec3& t = rbt.getTranslation();
        const Quat& r = rbt.getRotation();
        f << t[0] << ' ' << t[1] << ' ' << t[2] << ' '
        << r[0] << ' ' << r[1] << ' ' << r[2] << ' ' << r[3] << '\n';
      }
    }
  }

  int getNumKeyFrames() const {
    return keyFrames_.size();
  }

  int getNumRbtNodes() const {
    return nodes_.size();
  }

  // t can be in the range [0, keyFrames_.size()-3]. Fractional amount like 1.5 is allowed.
  void animate(double t) {
    if (t < 0 || t > keyFrames_.size() - 3)
      throw runtime_error("Invalid animation time parameter. Must be in the range [0, numKeyFrames - 3]");

    t += 1; // interpret the key frames to be at t= -1, 0, 1, 2, ...
    const int integralT = int(floor(t));
    const double fraction = t - integralT;

    KeyFrameIter f1 = getNthKeyFrame(integralT);
    KeyFrameIter f0 = f1; 
    KeyFrameIter f2 = f1; 
    KeyFrameIter f3 = f1; 
    --f0;
    ++f2; 
    ++f3; 
    ++f3;  

    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      nodes_[i]->setRbt(lerp((*f0)[i], (*f1)[i], (*f2)[i], (*f3)[i], fraction, int(floor(t))));
    }
  }

  KeyFrameIter keyFramesBegin() {
    return keyFrames_.begin();
  }

  KeyFrameIter keyFramesEnd() {
    return keyFrames_.end();
  }

  KeyFrameIter getNthKeyFrame(int n) {
    KeyFrameIter frameIter = keyFrames_.begin();
    advance(frameIter, n);
    return frameIter;
  }

  void deleteKeyFrame(KeyFrameIter keyFrameIter) {
    keyFrames_.erase(keyFrameIter);
  }

  void pullKeyFrameFromSg(KeyFrameIter keyFrameIter) {
    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      (*keyFrameIter)[i] = nodes_[i]->getRbt();
    }
  }

  void pushKeyFrameToSg(KeyFrameIter keyFrameIter) {
    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      nodes_[i]->setRbt((*keyFrameIter)[i]);
    }
  }

  KeyFrameIter insertEmptyKeyFrameAfter(KeyFrameIter beforeFrame) {
    if (beforeFrame != keyFrames_.end())
      ++beforeFrame;

    KeyFrameIter frameIter = keyFrames_.insert(beforeFrame, KeyFrame());
    frameIter->resize(nodes_.size());
    return frameIter;
  }

};

static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback


static Animator g_animator;
static Animator::KeyFrameIter g_curKeyFrame;
static int g_curKeyFrameNum;

///////////////// END OF G L O B A L S //////////////////////////////////////////////////

static void initCylinderMeshes() {

  glEnable(GL_TEXTURE_2D);

  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);

  g_cylinderMesh.load("Cylinder_test.mesh");
  vector<VertexPNX> vertice_iter; 

  // TODO: Init the per vertex normal of g_bunnyMesh, using codes from asst7
  for (int i = 0; i < g_cylinderMesh.getNumVertices(); ++i) {
    g_cylinderMesh.getVertex(i).setNormal(Cvec3());
  }

  for (int i = 0; i < g_cylinderMesh.getNumFaces(); ++i) {
     Mesh::Face face_3_verts = g_cylinderMesh.getFace(i);
     Cvec3 normal = face_3_verts.getNormal(); 

     for (int j = 0; j < face_3_verts.getNumVertices(); ++j) {
       Mesh::Vertex cur_vertex = face_3_verts.getVertex(j);
       cur_vertex.setNormal(normalize(cur_vertex.getNormal() + normal));
     }
  } 

  // TODO: Initialize g_bunnyGeometry from g_bunnyMesh, similar to
  // what you did for asst7 ...
  for (int i = 0; i < g_cylinderMesh.getNumFaces(); ++i) {
    Mesh::Face face_3_verts = g_cylinderMesh.getFace(i);

    for (int j = 0; j < face_3_verts.getNumVertices(); ++j) {
      const Mesh::Vertex v = face_3_verts.getVertex(j);

      Cvec2 texture = Cvec2(0.0, 0.0);

      if (j == 0) {
        texture = Cvec2(0.01, 0);
      }
      else if (j == 1) {
        texture = Cvec2(0, 0.01);
      }

      vertice_iter.push_back(VertexPNX(v.getPosition(), v.getNormal(), texture));             
    }
  }
  g_cylinderGeometry.reset(new SimpleGeometryPNX()); 
  g_cylinderGeometry->upload(&vertice_iter[0], vertice_iter.size());
}

static void initGround() {
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initCylinder() {
  int ibLen, vbLen;
  getCylinderVbIbLen(5, 1, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeCylinder(1, 1, 1, 5, 1, vtx.begin(), idx.begin());
  g_cylinderTEST.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

enum ManipMode {
  ARCBALL_ON_PICKED,
  ARCBALL_ON_SKY,
  EGO_MOTION
};

static ManipMode getManipMode() {
  // if nothing is picked or the picked transform is the transfrom we are viewing from
  if (g_currentPickedRbtNode == NULL || g_currentPickedRbtNode == g_currentCameraNode) {
    if (g_currentCameraNode == g_skyNode && g_activeCameraFrame == WORLD_SKY)
      return ARCBALL_ON_SKY;
    else
      return EGO_MOTION;
  }
  else
    return ARCBALL_ON_PICKED;
}

static bool shouldUseArcball() {
  return getManipMode() != EGO_MOTION;
}

// The translation part of the aux frame either comes from the current
// active object, or is the identity matrix when
static RigTForm getArcballRbt() {
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    return getPathAccumRbt(g_world, g_currentPickedRbtNode);
  case ARCBALL_ON_SKY:
    return RigTForm();
  case EGO_MOTION:
    return getPathAccumRbt(g_world, g_currentCameraNode);
  default:
    throw runtime_error("Invalid ManipMode");
  }
}

static void updateArcballScale() {
  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  double depth = arcballEye.getTranslation()[2];
  if (depth > -CS175_EPS)
    g_arcballScale = 0.02;
  else
    g_arcballScale = getScreenToEyeScale(depth, g_frustFovY, g_windowHeight);
}

static void drawArcBall(Uniforms& uniforms) {
  // switch to wire frame mode
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  Matrix4 MVM = rigTFormToMatrix(arcballEye) * Matrix4::makeScale(Cvec3(1, 1, 1) * g_arcballScale * g_arcballScreenRadius);
  sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

  g_arcballMat->draw(*g_sphere, uniforms);

  // switch back to solid mode
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

static void drawStuff(bool picking) {
  cout << "drawing stuff! " << animating << endl; 
  // Declare an empty uniforms
  Uniforms uniforms;

  // if we are not translating, update arcball scale
  if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && !g_mouseRClickButton && g_spaceDown)))
    updateArcballScale();

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

  const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currentCameraNode);
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
  const Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();

  uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(light1, 1)));
  uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(light2, 1)));

  if (!picking && !animating) {
    Drawer drawer(invEyeRbt, uniforms);
    g_world->accept(drawer);

    if (g_displayArcball && shouldUseArcball())
      drawArcBall(uniforms);
  }
  else if (animating) {
          cout << "Drawing iteration " << animate_level << endl; 

    TreeDrawer drawer(invEyeRbt, uniforms, animate_level);
    g_world->accept(drawer);


  }
  else {
    Picker picker(invEyeRbt, uniforms);
    // set overiding material to our picking material
    g_overridingMaterial = g_pickingMat;

    g_world->accept(picker);

    // unset the overriding material
    g_overridingMaterial.reset();

    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    if (g_currentPickedRbtNode == g_groundNode)
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>(); // set to NULL

    //cout << (g_currentPickedRbtNode ? "Part picked" : "No part picked") << endl;
  }
}

static void display() {
  // No more glUseProgram

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(false); // no more curSS

  glutSwapBuffers();

  checkGlErrors();
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // No more glUseProgram
  drawStuff(true); // no more curSS

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

bool interpolateAndDisplay(float t) {
  if (t > g_animator.getNumKeyFrames() - 3)
    return true;
  g_animator.animate(t);
  return false;
}

static void treeGrowAnimateTimerCallback(int ms) {
  double t = (double)ms / g_msBetweenKeyFrames;
  bool endReached = interpolateAndDisplay(t);
  if (animating && animate_level < tree_height) {
    glutTimerFunc(1000/g_animateFramesPerSecond, treeGrowAnimateTimerCallback, ms + 1000/g_animateFramesPerSecond);
    animate_level++; 
  }
  else {
    cerr << "Finished playing animation" << endl;
    animating = false; 
    animate_level = 3; 
  }
  display();
}

static void leafAnimateTimerCallback(int ms) {

  cout << "Animating" << endl; 
  double t = (double)ms / g_msBetweenKeyFrames;
  bool endReached = interpolateAndDisplay(t);
  if (simLeaves) {
    glutTimerFunc(1000/g_animateFramesPerSecond, leafAnimateTimerCallback, ms + 1000/g_animateFramesPerSecond);
  }
  else {
    cerr << "Finished playing animation" << endl;
  }
  RigTForm cur_trans = leafTransformNode->getRbt(); 

  int rand_num = rand() % 3; 
  if (rand_num == 0) {
    leafTransformNode->setRbt(RigTForm(Cvec3(), cur_trans.getRotation() + Quat::makeYRotation( rand() % 180 )));
  }
  else if (rand_num == 1) {
    leafTransformNode->setRbt(RigTForm(Cvec3(), cur_trans.getRotation() + Quat::makeXRotation( rand() % 180 )));
  }
  else {
    leafTransformNode->setRbt(RigTForm(Cvec3(), cur_trans.getRotation() + Quat::makeZRotation( rand() % 180 )));
  }
  display(); 
}


static void animateTimerCallback(int ms) {
  double t = (double)ms / g_msBetweenKeyFrames;
  bool endReached = interpolateAndDisplay(t);
  if (g_playingAnimation && !endReached) {
    glutTimerFunc(1000/g_animateFramesPerSecond, animateTimerCallback, ms + 1000/g_animateFramesPerSecond);
  }
  else {
    cerr << "Finished playing animation" << endl;
    g_curKeyFrame = g_animator.keyFramesEnd();
    advance(g_curKeyFrame, -2);
    g_animator.pushKeyFrameToSg(g_curKeyFrame);
    g_playingAnimation = false;

    g_curKeyFrameNum = g_animator.getNumKeyFrames() - 2;
    cerr << "Now at frame [" << g_curKeyFrameNum << "]" << endl;
  }
  display();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  g_arcballScreenRadius = max(1.0, min(h, w) * 0.25);
  updateFrustFovY();
  glutPostRedisplay();
}

static Cvec3 getArcballDirection(const Cvec2& p, const double r) {
  double n2 = norm2(p);
  if (n2 >= r*r)
    return normalize(Cvec3(p, 0));
  else
    return normalize(Cvec3(p, sqrt(r*r - n2)));
}

static RigTForm moveArcball(const Cvec2& p0, const Cvec2& p1) {
  const Matrix4 projMatrix = makeProjectionMatrix();
  const RigTForm eyeInverse = inv(getPathAccumRbt(g_world, g_currentCameraNode));
  const Cvec3 arcballCenter = getArcballRbt().getTranslation();
  const Cvec3 arcballCenter_ec = Cvec3(eyeInverse * Cvec4(arcballCenter, 1));

  if (arcballCenter_ec[2] > -CS175_EPS)
    return RigTForm();

  Cvec2 ballScreenCenter = getScreenSpaceCoord(arcballCenter_ec,
                                               projMatrix, g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
  const Cvec3 v0 = getArcballDirection(p0 - ballScreenCenter, g_arcballScreenRadius);
  const Cvec3 v1 = getArcballDirection(p1 - ballScreenCenter, g_arcballScreenRadius);

  return RigTForm(Quat(0.0, v1[0], v1[1], v1[2]) * Quat(0.0, -v0[0], -v0[1], -v0[2]));
}

static RigTForm doMtoOwrtA(const RigTForm& M, const RigTForm& O, const RigTForm& A) {
  return A * M * inv(A) * O;
}

static RigTForm getMRbt(const double dx, const double dy) {
  RigTForm M;

  if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) {
    if (shouldUseArcball())
      M = moveArcball(Cvec2(g_mouseClickX, g_mouseClickY), Cvec2(g_mouseClickX + dx, g_mouseClickY + dy));
    else
      M = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
  }
  else {
    double movementScale = getManipMode() == EGO_MOTION ? 0.02 : g_arcballScale;
    if (g_mouseRClickButton && !g_mouseLClickButton) {
      M = RigTForm(Cvec3(dx, dy, 0) * movementScale);
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && g_spaceDown)) {
      M = RigTForm(Cvec3(0, 0, -dy) * movementScale);
    }
  }

  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    break;
  case ARCBALL_ON_SKY:
    M = inv(M);
    break;
  case EGO_MOTION:
    if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) // only invert rotation
      M = inv(M);
    break;
  }
  return M;
}

static RigTForm makeMixedFrame(const RigTForm& objRbt, const RigTForm& eyeRbt) {
  return transFact(objRbt) * linFact(eyeRbt);
}

// l = w X Y Z
// o = l O
// a = w A = l (Z Y X)^1 A = l A'
// o = a (A')^-1 O
//   => a M (A')^-1 O = l A' M (A')^-1 O

static void motion(const int x, const int y) {
  if (!g_mouseClickDown)
    return;

  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  const RigTForm M = getMRbt(dx, dy);   // the "action" matrix

  // the matrix for the auxiliary frame (the w.r.t.)
  RigTForm A = makeMixedFrame(getArcballRbt(), getPathAccumRbt(g_world, g_currentCameraNode));

  shared_ptr<SgRbtNode> target;
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    target = g_currentPickedRbtNode;
    break;
  case ARCBALL_ON_SKY:
    target = g_skyNode;
    break;
  case EGO_MOTION:
    target = g_currentCameraNode;
    break;
  }

  A = inv(getPathAccumRbt(g_world, target, 1)) * A;

  target->setRbt(doMtoOwrtA(M, target->getRbt(), A));

  g_mouseClickX += dx;
  g_mouseClickY += dy;
  glutPostRedisplay();  // we always redraw if we changed the scene
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

  if (g_pickingMode && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    pick();
    g_pickingMode = false;
    g_displayArcball = true; 
    cerr << "Picking mode is off" << endl;
    glutPostRedisplay(); // request redisplay since the arcball will have moved
  }
  glutPostRedisplay();
}

static void keyboardUp(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ' ':
    g_spaceDown = false;
    break;
  }
  glutPostRedisplay();
}

static void keyboard(const unsigned char key, const int x, const int y) {
  ostringstream ss; 

  switch (key) {
  case ' ':
    g_spaceDown = true;
    break;
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "p\t\tUse mouse to pick a part to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n"
    << "a\t\tToggle display arcball\n"
    << "w\t\tWrite animation to animation.txt\n"
    << "i\t\tRead animation from animation.txt\n"
    << "c\t\tCopy frame to scene\n"
    << "u\t\tCopy sceneto frame\n"
    << "n\t\tCreate new frame after current frame and copy scene to it\n"
    << "d\t\tDelete frame\n"
    << ">\t\tGo to next frame\n"
    << "<\t\tGo to prev. frame\n"
    << "y\t\tPlay/Stop animation\n"
    << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  // case 'f':
  //   g_activeShader = (g_activeShader + 1) % g_numRegularShaders;
  //   break;
  case 'v':
  {
    shared_ptr<SgRbtNode> viewers[] = {g_skyNode};
    for (int i = 0; i < 3; ++i) {
      if (g_currentCameraNode == viewers[i]) {
        g_currentCameraNode = viewers[(i+1)%3];
        break;
      }
    }
  }
  break;
  case 'p':
    g_pickingMode = !g_pickingMode;
    if (g_pickingMode) 
      g_displayArcball = false;
    cerr << "Picking mode is " << (g_pickingMode ? "on" : "off") << endl;
    break;
  case 'm':
    g_activeCameraFrame = SkyMode((g_activeCameraFrame+1) % 2);
    cerr << "Editing sky eye w.r.t. " << (g_activeCameraFrame == WORLD_SKY ? "world-sky frame\n" : "sky-sky frame\n") << endl;
    break;
  case 'a':
    g_displayArcball = !g_displayArcball;
    break;
  case 'u':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }

    if (g_curKeyFrame == g_animator.keyFramesEnd()) { // only possible when frame list is empty
      cerr << "Create new frame [0]."  << endl;
      g_curKeyFrame = g_animator.insertEmptyKeyFrameAfter(g_animator.keyFramesBegin());
      g_curKeyFrameNum = 0;
    }
    cerr << "Copying scene graph to current frame [" << g_curKeyFrameNum << "]" << endl;
    g_animator.pullKeyFrameFromSg(g_curKeyFrame);
    break;
  case 'n':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_animator.getNumKeyFrames() != 0)
      ++g_curKeyFrameNum;
    g_curKeyFrame = g_animator.insertEmptyKeyFrameAfter(g_curKeyFrame);
    g_animator.pullKeyFrameFromSg(g_curKeyFrame);
    cerr << "Create new frame [" << g_curKeyFrameNum << "]" << endl;
    break;
  case 'c':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      cerr << "Loading current key frame [" << g_curKeyFrameNum << "] to scene graph" << endl;
      g_animator.pushKeyFrameToSg(g_curKeyFrame);
    }
    else {
      cerr << "No key frame defined" << endl;
    }
    break;
  case 'd':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      Animator::KeyFrameIter newCurKeyFrame = g_curKeyFrame;
      cerr << "Deleting current frame [" << g_curKeyFrameNum << "]" << endl;;
      if (g_curKeyFrame == g_animator.keyFramesBegin()) {
        ++newCurKeyFrame;
      }
      else {
        --newCurKeyFrame;
        --g_curKeyFrameNum;
      }
      g_animator.deleteKeyFrame(g_curKeyFrame);
      g_curKeyFrame = newCurKeyFrame;
      if (g_curKeyFrame != g_animator.keyFramesEnd()) {
        g_animator.pushKeyFrameToSg(g_curKeyFrame);
        cerr << "Now at frame [" << g_curKeyFrameNum << "]" << endl;
      }
      else
        cerr << "No frames defined" << endl;
    }
    else {
      cerr << "Frame list is now EMPTY" << endl;
    }
    break;
  case '>':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      if (++g_curKeyFrame == g_animator.keyFramesEnd())
        --g_curKeyFrame;
      else {
        ++g_curKeyFrameNum;
        g_animator.pushKeyFrameToSg(g_curKeyFrame);
        cerr << "Stepped forward to frame [" << g_curKeyFrameNum <<"]" << endl;
      }
    }
    break;
  case '<':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesBegin()) {
      --g_curKeyFrame;
      --g_curKeyFrameNum;
      g_animator.pushKeyFrameToSg(g_curKeyFrame);
      cerr << "Stepped backward to frame [" << g_curKeyFrameNum << "]" << endl;
    }
    break;
  case 'w':
    cerr << "Writing animation to animation.txt\n";
    g_animator.saveAnimation("animation.txt");
    break;
  case 'i':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    cerr << "Reading animation from animation.txt\n";
    g_animator.loadAnimation("animation.txt");
    g_curKeyFrame = g_animator.keyFramesBegin();
    cerr << g_animator.getNumKeyFrames() << " frames read.\n";
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      g_animator.pushKeyFrameToSg(g_curKeyFrame);
      cerr << "Now at frame [0]" << endl;
    }
    g_curKeyFrameNum = 0;
    break;
  case '-':
    g_msBetweenKeyFrames = min(g_msBetweenKeyFrames + 100, 10000);
    cerr << g_msBetweenKeyFrames << " ms between keyframes.\n";
    break;
  case '+':
    g_msBetweenKeyFrames = max(g_msBetweenKeyFrames - 100, 100);
    cerr << g_msBetweenKeyFrames << " ms between keyframes.\n";
    break;
  case 'y':
    if (!g_playingAnimation) {
      if (g_animator.getNumKeyFrames() < 4) {
        cerr << " Cannot play animation with less than 4 keyframes." << endl;
      }
      else {
        g_playingAnimation = true;
        cerr << "Playing animation... "<< endl;
        animateTimerCallback(0);
      }
    }
    else {
      cerr << "Stopping animation... " << endl;
      g_playingAnimation = false;
    }
    break; 
  case '1':
    cout << "Num iterations increased" << endl;
    num_iterations++; 
    // // NOT the correct way to do this. 
    // initScene(); 
    // initAnimation();
    //g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2)))); 
    //g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));  g
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break; 
  case '2':
    cout << "Num iterations decreased" << endl;
    num_iterations--; 
    if (num_iterations < 0) {
      num_iterations = 0; 
    }
    // NOT the correct way to do this.
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break; 
  case '3':
    selected_tree++; 
    selected_tree %= 6; 
    ss << selected_tree; 
    tree_lookup = "Lsystems/l" + ss.str() + ".txt"; 
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break;
  case '4':
    branch_thickness += 0.1; 
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break;
  case '5':
    branch_thickness -= 0.1; 
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break;
  case '6':
    branch_length *= 2; 
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break;
  case '7':
    branch_length /= 2; 
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break;
  case '8':
    leaves_on++; 
    leaves_on%=2; 
    g_world->removeChild(g_treeNode);
    g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));     
    constructTree(g_treeNode, g_barkMat, g_leafMat);
    g_world->addChild(g_treeNode);
    break;
  case '9': 
    simLeaves = !simLeaves; 
    if (simLeaves) {
      g_world->removeChild(g_treeNode);
      g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));   
      constructTree(g_treeNode, g_barkMat, g_leafMat);
      g_world->addChild(g_treeNode);
    }
    leafAnimateTimerCallback(0);
    break;
  case '0':
    animating = !animating; 
    treeGrowAnimateTimerCallback(0);
    cout << "Printing tree " << endl;  
    break; 
  }

  // Sanity check that our g_curKeyFrameNum is in sync with the g_curKeyFrame
  if (g_animator.getNumKeyFrames() > 0)
    assert(g_animator.getNthKeyFrame(g_curKeyFrameNum) == g_curKeyFrame);

  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
#ifdef __MAC__
  glutInitDisplayMode(GLUT_3_2_CORE_PROFILE|GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH); // core profile flag is required for GL 3.2 on Mac
#else
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
#endif
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 7");                       // title the window

  glutIgnoreKeyRepeat(true);                              // avoids repeated keyboard calls when holding space to emulate middle mouse

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
  glutKeyboardUpFunc(keyboardUp);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");
  Material bark("./shaders/basic-gl3-textured.vshader", "./shaders/diffuse-gl3-textured.fshader");
  Material grass("./shaders/basic-gl3-textured.vshader", "./shaders/diffuse-gl3-textured.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

  g_greenDiffuseMat.reset(new Material(diffuse));
  g_greenDiffuseMat->getUniforms().put("uColor", Cvec3f(0,1,0));

  // normal mapping material
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("./textures/Fieldstone.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("./textures/FieldstoneNormal.ppm", false)));

  //g_barkMat.reset(new Material(bark));
  g_barkMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3-less-shiny.fshader"));
  g_barkMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("./textures/5745.ppm", true)));
  g_barkMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("./textures/5745-normal.ppm", false)));
  g_barkMat->getRenderStates().blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) // set blending mode
  .enable(GL_BLEND) // enable blending
  .disable(GL_CULL_FACE); // disable culling

  g_leafMat.reset(new Material("./shaders/leaf.vshader", "./shaders/bunny-shell-gl3.fshader"));
  g_leafMat->getUniforms().put("uTexShell", shared_ptr<ImageTexture>(new ImageTexture("./textures/leaf.ppm", true)));
  g_leafMat->getRenderStates()
  .blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) // set blending mode
  .enable(GL_BLEND) // enable blending
  .disable(GL_CULL_FACE); // disable culling

  g_grassMat.reset(new Material(grass));
  //g_grassMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  //g_grassMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("textures/Grass.0002.ppm", true)));
  g_grassMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("./textures/22_DIFFUSE.ppm", true)));
  //g_grassMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("./textures/22_NORMAL.ppm", false)));

  g_brownDiffuseMat.reset(new Material(bark));
  g_brownDiffuseMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("./textures/5745.ppm", true)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 0));


  shared_ptr<ImageTexture> barkTexture(new ImageTexture("./textures/5745.ppm", true));
  g_cylinderMat.reset(new Material("./shaders/bunny-shell-gl3.vshader", "./shaders/bunny-shell-gl3.fshader"));
  g_cylinderMat->getUniforms().put("uTexShell", barkTexture);
  g_cylinderMat->getRenderStates().disable(GL_CULL_FACE); // disable culling

  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
  initGround();
  initCubes();
  initSphere();
  initCylinder(); 
  //initLeaf(); 
  //initCylinder(); 
  initCylinderMeshes();
}

static int computeTreeHeight( shared_ptr<SgTransformNode> start, int depth ) {
  if(start->getNumChildren() > 0) {
    if(depth+1 > tree_height)
      tree_height = depth+1;

    for(int i = 0; i < start->getNumChildren(); ++i) {
      shared_ptr<SgNode> s = start->getChild(i);
      shared_ptr<SgTransformNode> ptr(dynamic_pointer_cast<SgTransformNode>(s));
      if(ptr)
      {
        computeTreeHeight( ptr, depth+1);
      }
    }
  }
}

static void constructTree(shared_ptr<SgTransformNode> base, shared_ptr<Material> trunk_material, shared_ptr<Material> leaf_material) {
  LSystem* check = new LSystem(tree_lookup);
  std::string tmp = check->gen_string(check->axiom, 0, num_iterations);

  jointNodes.clear(); 
  jointNodes.push_back(base);

  vector< int > jointIds;
  int cur_jointId = 0; 
  int highest_jointId = 0; 
  jointIds.push_back(cur_jointId);

  vector < double > thickness;
  double cur_thickness = 0.1 * branch_thickness; 
  thickness.push_back(cur_thickness);

  leavesToAdd = priority_queue <int> (); 
  int rotate = 0; 

  int first = 0; 
  //tree_height = 0; 

  Cvec3 last_translation = Cvec3(); 
  vector < Cvec3 > all_translations; 
  all_translations.push_back(last_translation);

  for (int i = 0; i < tmp.length(); ++i) {
    if (tmp.substr(i, 1) == "F") {

      int r3 = rand() % 3; 
      if (rand() % 2 == 0) 
        r3 *= -1; 


      
      //if (cur_thickness > 0.025) {
        jointNodes[cur_jointId]->addChild(shared_ptr<SgGeometryShapeNode>(
                           new MyShapeNode(g_cylinderTEST, 
                                          g_barkMat,
                                          //g_cylinderMat,
                                          //g_grassMat,
                                          //g_brownDiffuseMat,
                                          last_translation, //lastLocation.getTranslation(),
                                          Cvec3(0, 0, 0),
                                          Cvec3(cur_thickness,branch_length /*0.075*/, cur_thickness )))); //cur_thickness))));
      // }
      // else {
      // jointNodes[cur_jointId]->addChild(shared_ptr<SgGeometryShapeNode>(
      //                      new MyShapeNode(g_cylinderGeometry, 
      //                                     //g_barkMat,
      //                                     g_cylinderMat,
      //                                     //g_grassMat,
      //                                     //g_brownDiffuseMat,
      //                                     Cvec3(0,0,0), //lastLocation.getTranslation(),
      //                                     Cvec3(90, 0, 0),
      //                                     Cvec3(cur_thickness,cur_thickness,branch_length * 0.075))));
      // }
      int r1 = rand() % 5;
      int r2 = rand() % 2 - 1; 
      if (r1 == 0 && rotate == 1 && cur_thickness < 0.025) {
        leavesToAdd.push(cur_jointId);
        // jointNodes[cur_jointId]->addChild(shared_ptr<SgGeometryShapeNode>(
        //                    new MyShapeNode(g_cube,
        //                                   leaf_material,
        //                                   Cvec3(0,0,0), //lastLocation.getTranslation(),
        //                                   Cvec3(rand() % 15, 0,0 ),
        //                                   Cvec3(0.1,0.2,0.001))));
      }

      // shared_ptr<SgTransformNode> transformNode;
      // transformNode.reset(new SgRbtNode(RigTForm(Cvec3(0,branch_length * 0.075,0))));
      // jointNodes.push_back( transformNode );
      // jointNodes[cur_jointId]->addChild(transformNode);
      // highest_jointId++;
      // cur_jointId = highest_jointId;
      last_translation += Cvec3(0,branch_length * 0.075,0) ; 


    } 
    else if (tmp.substr(i, 1) == "+") {
      rotate = 1; 
      shared_ptr<SgTransformNode> transformNode;
      int r1 = rand() % 3;
      Cvec3 adjustment = Cvec3(); 
      Quat rotatation = Quat(); 
      int r2 = rand() % 2;
      if (r2 == 0) {
        r2 = -1; 
      } 
      if (r1 == 0) {
        rotatation = Quat::makeZRotation(30 + r2 * (rand() % 5));
        adjustment = Cvec3(-1.0/10 * cur_thickness,1.0/2.0 * cur_thickness,-1.0/10 * cur_thickness); 
      }
      else if (r1 == 1) {
        rotatation = Quat::makeYRotation(30 + r2 * (rand() % 5));
        adjustment = Cvec3(-1.0/2.0 * cur_thickness,1.0/2.0 * cur_thickness,-1.0/10 * cur_thickness); 
      }
      else {
        rotatation = Quat::makeXRotation(30 + r2 * (rand() % 5));
        adjustment = Cvec3(-1.0/20 * cur_thickness,-1 * cur_thickness,-1.0/10 * cur_thickness); 
      }
      transformNode.reset(new SgRbtNode(RigTForm(last_translation + adjustment, rotatation)));
      jointNodes.push_back( transformNode );
      jointNodes[cur_jointId]->addChild(transformNode);
      highest_jointId++;
      cur_jointId = highest_jointId;
      if (first == 0) {
        tree_base = transformNode;
        first = 1; 
      }
      last_translation = Cvec3(); 

    } 
    else if (tmp.substr(i, 1) == "-") {
      shared_ptr<SgTransformNode> transformNode;
      int r1 = rand() % 3;
      Cvec3 adjustment = Cvec3(); 
      Quat rotatation = Quat(); 
      int r2 = rand() % 2;
      if (r2 == 0) {
        r2 = -1; 
      } 
      if (r1 == 0) {
        rotatation = Quat::makeZRotation(-30 + r2 * (rand() % 5));
        adjustment = Cvec3(-1.0/20 * cur_thickness,-1* cur_thickness,-1.0/20 * cur_thickness); 
      }
      else if (r1 == 1) {
        rotatation = Quat::makeYRotation(-30 + r2 * (rand() % 5));
        adjustment = Cvec3(-1.0/20 * cur_thickness,-1* cur_thickness,-1.0/20 * cur_thickness); 
      }
      else {
        rotatation = Quat::makeXRotation(-30 + r2 * (rand() % 5));
        adjustment = Cvec3(-1.0/20 * cur_thickness,-1* cur_thickness,-1.0/20 * cur_thickness); 
      }
      transformNode.reset(new SgRbtNode(RigTForm(last_translation + adjustment, rotatation)));
      jointNodes.push_back( transformNode );
      jointNodes[cur_jointId]->addChild(transformNode);
      highest_jointId++;
      cur_jointId = highest_jointId;
      if (first == 0) {
        tree_base = transformNode;
        first = 1; 
      }
      last_translation = Cvec3(); 
    } 
    else if (tmp.substr(i, 1) == "[") {
      jointIds.push_back(cur_jointId);
      cur_thickness = MAX((thickness.back() * 2.0 / 3.0), 0.01); 
      thickness.push_back(cur_thickness);
      all_translations.push_back(last_translation);
    }
    else if (tmp.substr(i, 1) == "]") {
      cur_jointId = jointIds.back();
      jointIds.pop_back(); 
      cur_thickness = thickness.back(); 
      thickness.pop_back(); 
      last_translation = all_translations.back(); 
      all_translations.pop_back(); 
    }
    else if (tmp.substr(i, 1) == "X") {

    }
    else {
    }
  }

  if (simLeaves) {
    while (!leavesToAdd.empty() and leaves_on == 0) {
      //cout << "leaf!" << endl;
      int leaf_id = leavesToAdd.top(); 
      leavesToAdd.pop(); 

      //RigTForm cur_trans = leafTransformNode->getRbt(); 
      //transformNode->setRbt(RigTForm(Cvec3(), cur_trans.getRotation() + Quat::makeYRotation(10)));
      leafTransformNode->addChild(shared_ptr<SgGeometryShapeNode>(
                         new MyShapeNode(g_cube,
                                        g_leafMat,
                                        Cvec3(0.1,0.01,0), //lastLocation.getTranslation(),
                                        Cvec3(15,15,90),
                                        Cvec3(0.2, 0.2,0.00001))));
      jointNodes[leaf_id]->addChild(leafTransformNode);

      // jointNodes[leaf_id]->addChild(shared_ptr<SgGeometryShapeNode>(
      //                    new MyShapeNode(g_cube,
      //                                   g_leafMat,
      //                                   Cvec3(0.1,0.01,0), //lastLocation.getTranslation(),
      //                                   Cvec3(15,15,90),
      //                                   Cvec3(0.1, 0.1,0.00001))));
    }
  }
  else {
    while (!leavesToAdd.empty() and leaves_on == 0) {
      //cout << "leaf!" << endl;
      int leaf_id = leavesToAdd.top(); 
      leavesToAdd.pop(); 

      jointNodes[leaf_id]->addChild(shared_ptr<SgGeometryShapeNode>(
                         new MyShapeNode(g_cube,
                                        g_leafMat,
                                        Cvec3(0.1,0.01,0), //lastLocation.getTranslation(),
                                        Cvec3(15,15,90),
                                        Cvec3(0.1, 0.1,0.00001))));
    }
  }

  //tree_height = highest_jointId; 
  tree_height = 0;
  computeTreeHeight( base, 0 );
  cout << tree_height << endl; 
}

static void initScene() {
  leafTransformNode.reset(new SgRbtNode(RigTForm()));

  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 1, 6.0))));


  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, g_grassMat, Cvec3(0, g_groundY, 0))));

  g_light1Node.reset(new SgRbtNode(RigTForm(g_light1, Quat())));
  g_light2Node.reset(new SgRbtNode(RigTForm(g_light2, Quat())));

  g_light1Node->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_sphere, g_lightMat, Cvec3())));
  g_light2Node->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_sphere, g_lightMat, Cvec3())));

  //g_cylinderNode.reset(new SgRbtNode());
  //g_cylinderNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_cylinderGeometry, g_cylinderMat)));

  g_treeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, -2, -2))));
  constructTree(g_treeNode, g_barkMat, g_leafMat);

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_light1Node);
  g_world->addChild(g_light2Node);
  g_world->addChild(g_treeNode);
  //g_world->addChild(g_cylinderNode);
  g_currentCameraNode = g_skyNode;
}

static void initAnimation() {
  g_animator.attachSceneGraph(g_world);
  g_curKeyFrame = g_animator.keyFramesBegin();
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    // on Mac, we shouldn't use GLEW.

#ifndef __MAC__
    glewInit(); // load the OpenGL extensions
#endif

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.5") << endl;

#ifndef __MAC__
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
#endif

    initGLState();
    initMaterials(); 
    initGeometry();
    initScene();
    initAnimation();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
