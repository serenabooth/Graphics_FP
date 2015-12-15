#ifndef DRAWER_H
#define DRAWER_H

#include <vector>

#include "uniforms.h"
#include "scenegraph.h"
#include "asstcommon.h"
using namespace std;

class Drawer : public SgNodeVisitor {
protected:
  std::vector<RigTForm> rbtStack_;
  Uniforms& uniforms_;
public:
  Drawer(const RigTForm& initialRbt, Uniforms& uniforms)
    : rbtStack_(1, initialRbt)
    , uniforms_(uniforms) {}

  virtual bool visit(SgTransformNode& node) {
    rbtStack_.push_back(rbtStack_.back() * node.getRbt());
    return true;
  }

  virtual bool postVisit(SgTransformNode& node) {
    rbtStack_.pop_back();
    return true;
  }

  virtual bool visit(SgShapeNode& shapeNode) {
    const Matrix4 MVM = rigTFormToMatrix(rbtStack_.back()) * shapeNode.getAffineMatrix();
    sendModelViewNormalMatrix(uniforms_, MVM, normalMatrix(MVM));
    shapeNode.draw(uniforms_);
    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    return true;
  }

  Uniforms& getUniforms() {
    return uniforms_;
  }
};



class TreeDrawer : public Drawer {
protected:
  //std::vector<RigTForm> rbtStack_;
  //Uniforms& uniforms_;
  int limit_; 
  int depth_stack_;
public:
  TreeDrawer(const RigTForm& initialRbt, Uniforms& uniforms, int limit)
    : Drawer( initialRbt, uniforms ) 
    {
      limit_ = limit;
      depth_stack_ = 0;
    }

  virtual bool visit(SgTransformNode& node) {
    rbtStack_.push_back(rbtStack_.back() * node.getRbt());
    depth_stack_++; 
    return true;
  }

  virtual bool postVisit(SgTransformNode& node) {
    rbtStack_.pop_back();
    depth_stack_--; 
    return true;
  }

  virtual bool visit(SgShapeNode& shapeNode) {
    if (depth_stack_ < limit_) {
      const Matrix4 MVM = rigTFormToMatrix(rbtStack_.back()) * shapeNode.getAffineMatrix();
      sendModelViewNormalMatrix(uniforms_, MVM, normalMatrix(MVM));
      shapeNode.draw(uniforms_);
    }
    else {
      //cout << "Too deep " << endl; 
    }
    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    return true;
  }

  Uniforms& getUniforms() {
    return uniforms_;
  }
};


#endif


