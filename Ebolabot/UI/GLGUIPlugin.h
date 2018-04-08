#ifndef GL_GUI_PLUGIN_H
#define GL_GUI_PLUGIN_H

#include <KrisLibrary/camera/camera.h>
#include <KrisLibrary/camera/viewport.h>
#include <string>

/** @brief A generic base class for hooks into the OpenGL event loop.
 * A subclass will use this to gain functionality with a GUI.
 *
 * A GUI class will use this to implement feedback into a TaskGenerator.
 * The GUI has the responsibility of setting up the camera matrix and
 * viewport before calling RenderWorld() and RenderScreen().
 *
 * The plugin can request a redraw using the Refresh() function, and can
 * implement a timed idle loop by setting SleepIdle() during the OnIdle()
 * function at 1.0 / the desired rate.
 */
class GLGUIPlugin
{
 public:
  GLGUIPlugin():wantsRefresh(false),sleepIdleTime(Math::Inf) {}
  virtual ~GLGUIPlugin() {}
  virtual void Start() {}
  virtual void Stop() {}
  virtual void RenderWorld() {}
  virtual void RenderScreen() {}
  virtual bool OnIdle() { return false; }
  virtual bool OnViewportChange() { return false; }
  virtual bool OnCameraChange() { return false; }
  virtual bool OnButtonPress(const std::string& button) { return false; }
  virtual bool OnButtonToggle(const std::string& button,int checked) { return false; }
  virtual bool OnWidgetValue(const std::string& widget,const std::string& value) { return false; }
  virtual bool OnMouseClick(int button,int state,int mx,int my) { return false; }
  virtual bool OnMouseMove(int mx,int my) { return false; }
  virtual bool OnMouseWheel(int dwheel) { return false; }
  virtual bool OnScroll(int dy) { return false; }
  virtual bool OnKeyDown(const std::string& key) { return false; }
  virtual bool OnKeyUp(const std::string& key) { return false; }
  virtual bool OnSpaceball(const Math3D::RigidTransform& T) { return false; }
  virtual bool OnDevice(const std::string& name,const std::string& data) { return false; }
  void Refresh() { wantsRefresh = true; }
  void SleepIdle(double seconds=Math::Inf) { sleepIdleTime=seconds; }
  void ClickRay(float mx,float my,Math3D::Vector3& src,Math3D::Vector3& dir) const { viewport.getClickSource(mx,my,src);  viewport.getClickVector(mx,my,dir); }

  Camera::Viewport viewport;
  bool wantsRefresh;
  double sleepIdleTime;
};

#endif
