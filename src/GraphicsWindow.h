/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef AFF_GRAPHICSWINDOW_H
#define AFF_GRAPHICSWINDOW_H

#include "ComponentBase.h"

#include <RcsViewer.h>
#include <GraphNode.h>
#include <KeyCatcher.h>
#include <HUD.h>
#include <VertexArrayNode.h>

#include <utility>


namespace aff
{
/*! \brief Graphics window class.
 *
 *         The class publishes the following events:
 *         - Quit: When the 'q' key is pressed in the graphics window.
 *         - RenderCommand: When the 'x' key is pressed in the graphics
 *                          window. This should be removed.
 *
 *         The class subscribes to the following events:
 *         - RenderGraph: Adds or updates a RcsGraph node
 *         - RenderLines: Draws a set of 3d lines from a n x d array
 *         - RenderCommand: The following string tuples are supported:
 *           - "SetEnableMeshFactory" - "true, false"
 *           - "ShowLines" - "true, false"
 *           - "BackgroundColor" - name of color
 *           - "CameraTransform" - Stringized HTr of transform (row-major)
 *           - "ResetView": set camera transform to the one the viewer launched
 *           - "FieldOfView" - "fovx" or "fovx fovy": set camera's field of view
 *           - "hud" - "hide, show, toggle"
 *           - "setGhostMode" - Graph-ID
 *           - "unsetGhostMode" - Graph-ID
 *           - "toggleGraphicsModel" - Graph-ID
 *           - "togglePhysicsModel" - Graph-ID
 *           - "toggleCollisionModel" - Graph-ID
 *           - "toggleReferenceFrames" - Graph-ID
 *           - "show" - Graph-ID
 *           - "hide" - Graph-ID
 *           - "erase" - Graph-ID
 *           - "model_state" - Graph-ID
 *         - RenderClear: Clears all visual nodes
 *         - Print: Prints all added nodes
 *         - SetText: Sets text to hud
 *         - SetTextLine: Sets text to a given hud text line
 *         - AddText: Appends text to last line of hud
 *         - ClearText: Clears all text from the hud
 *         - SetObjectActivation: Clears deactivated bodies from viewer window
 *         - EmergencyStop: Shows "EmergencyStop" on hud
 *         - EmergencyRecover: Clears hud
 *         - Render: Only for synchronisation with event loop: Calls the
 *                   viewer's frame() function.
 *         - Start: Only for startWithStartEvent==true: Starts the viewer
 *                  thread.
 *         - SetNodeTransform: Node-name, transform
 *
 *         The class publishes the following events:
 *         - GraphicsWindowFeedback:
 *           - GraphNodeCreated: String of graph-id that has been added to the
 *             viewer's scene graph. This event exists since the generation of
 *             GraphNodes is realized in a thread. At the end of it, this event
 *             is published so that the subscribers can create dependent nodes
 *             if needed.
 *
 *         \todo:
 *         - This class uses some static maps. Make it a member to support
 *           several GraphicsWindows
 */
class GraphicsWindow : public ComponentBase, public Rcs::Viewer
{
public:

  /*! \brief Constructs a GraphicsWindow class.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] startWithStartEvent Starts the viewer thread with the "Start"
   *                                event. If false, the thread starts right
   *                                after construction. This is sometimes a bit
   *                                more helpful, since it allows to see what's
   *                                going on during initialization.
   * \param[in] syncWithEventLoop   If true, the viewer's frame call (where all
   *                                the rendering takes place) is triggered
   *                                by the "Render" event, and is not called
   *                                from a class-internal thread.
   * \param[in] simpleGraphics      Starts graphics without shadows and anti-
   *                                aliasing. Usually only needed for old
   *                                graphics cards.
   */
  GraphicsWindow(EntityBase* parent, bool startWithStartEvent=false,
                 bool syncWithEventLoop=false, bool simpleGraphics=false);

  /*! \brief Unsubscribes and deletes all previously allocated memory.
   *         Running hreads will be stopped, and all viewer nodes will be
   *         erased.
   */
  virtual ~GraphicsWindow();

  /*! \brief Returns the name of this component.
   *
   * \return "GraphicsWindow"
   */
  std::string getName() const;

  /*! \brief Registers a callback function that is called on each key press
   *         event with the given key.
   *
   * \param[in] key   Key that when pressed triggers the callback function
   * \param[in] cb    Callback function to be called when key is pressed
   * \param[in] description   String that is displayed when calling the
   *                          KeyCatcher print message
   * \param[in] group   Group that the description message is associated with.
   *                    This only structures the console output.
   */
  void setKeyCallback(char key, std::function<void(char)> cb,
                      std::string description = "",
                      std::string group = "Application");

  /*! \brief Removes a callback function from the viewer.
   *
   * \param[in] key   Key that the callback function is registered with.
   */
  void clearKeyCallback(char key);

  /*! \brief Returns the mutex that embraces the viewer's frame call. The frame
   *         call performs all the graphics rendering.
   *
   * \return Pointer to mutex
   */
  pthread_mutex_t* getFrameMtx() const;

  /*! \brief Returns a pointer to the graph with the given id, or NULL if no
   *         such id has been added to the GraphicsWindow.
   *
   * \param[in]   graphId   Id string that a graph has been added with
   * \return Pointer to graph corresponding to graphId
   */
  const RcsGraph* getGraphById(std::string graphId);

  /*! \brief Returns a pointer to the GraphNode with the given id, or NULL if
   *         no such id has been added to the GraphicsWindow.
   *
   * \param[in]   graphId   Id string that a graph has been added with
   * \return Pointer to GraphNode corresponding to graphId
   */
  Rcs::GraphNode* getGraphNodeById(std::string graphId, bool& notRealizedYet);

  /*! \brief Launch the viewer window, in case it has not already been launched.
    */
  virtual void start();

protected:

  virtual void frame();
  virtual void stop();
  virtual void subscribeAll(bool startWithStartEvent);
  virtual void handleKeys();
  virtual void print();

  void onRender(std::string graphId, const RcsGraph* other);
  void onReloadGraph(std::string graphId, const RcsGraph* other);
  void onRenderLines(const MatNd* array);
  void onRenderCommand(std::string graphId, std::string command);
  void onObjectActivation(std::string objectName, bool activation);
  void onObjectColor(std::string graphId, std::string objectName,
                     std::string color);
  void onObjectsAlpha(std::string objectName, double alpha);
  void onObjectAlpha(std::string whichGraph, std::string objectName,
                     double alpha);
  void onEmergencyStop();
  void onEmergencyRecover();
  void onAddNode(osg::ref_ptr<osg::Node> node);
  void onAddChildNode(osg::ref_ptr<osg::Node> node, std::string graphId,
                      std::string parent);
  void onRemoveNode(std::string graphId, std::string nodeName);
  void onSetNodeTransform(std::string nodeName, HTr transform);
  void setText(std::string text);
  void setTextLine(std::string text, int lineNum);
  void addText(std::string text);
  void updateText();
  void clearText();
  void clear();

  osg::ref_ptr<Rcs::KeyCatcher> keyCatcher;
  osg::ref_ptr<Rcs::HUD> hud;
  osg::ref_ptr<Rcs::VertexArrayNode> vertexNode;
  std::map<char, std::function<void(char)>> keyCallbacks;
  std::vector<std::string> hudText;
  mutable pthread_mutex_t frameMtx;
  bool synWithEventLoop;

private:

  GraphicsWindow(const GraphicsWindow&);
  GraphicsWindow& operator=(const GraphicsWindow&);
};

}

#endif   // AFF_GRAPHICSWINDOW_H
