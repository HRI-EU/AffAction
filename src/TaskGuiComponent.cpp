/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

*******************************************************************************/

#include "TaskGuiComponent.h"

#include <TaskWidget.h>
#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>



namespace aff
{

class TaskUpdateCallback : public TaskWidget::TaskChangeCallback
{
public:
  TaskUpdateCallback(TaskGuiComponent* gui_) : gui(gui_)
  {
  }

  virtual void callback()
  {
    gui->guiCallback();
  };

  TaskGuiComponent* gui;
};


TaskGuiComponent::TaskGuiComponent(EntityBase* parent,
                                   const Rcs::ControllerBase* controller_) :
  ComponentBase(parent), controller(*controller_), a_des(NULL),
  x_des(NULL), x_des_filt(NULL), x_curr(NULL), filt(NULL), gui(NULL), passive(false)
{
  this->a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
  this->x_curr  = MatNd_create(controller.getTaskDim(), 1);
  controller_->readActivationsFromXML(this->a_des);
  controller.computeX(this->x_curr);
  this->x_des = MatNd_clone(this->x_curr);
  this->x_des_filt = MatNd_clone(this->x_curr);

  const double tmc = 0.25;
  const double vmax = 0.4;
  this->filt = new Rcs::RampFilterND(this->x_curr->ele, tmc, vmax, parent->getDt(),
                                     controller.getTaskDim());

  pthread_mutex_init(&this->mtx, NULL);

  subscribe("Start", &TaskGuiComponent::start);
  subscribe("Stop", &TaskGuiComponent::stop);
  subscribe("ComputeKinematics", &TaskGuiComponent::setState);
  subscribe("InitFromState", &TaskGuiComponent::onInitFromState);
}

TaskGuiComponent::~TaskGuiComponent()
{
  delete this->gui;
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
  MatNd_destroy(this->x_des_filt);
  MatNd_destroy(this->x_curr);
  delete this->filt;
  pthread_mutex_destroy(&this->mtx);
}

void TaskGuiComponent::setPassive(bool value)
{
  this->passive = value;
}

void TaskGuiComponent::start()
{
  RLOG(0, "Start::start()");
  // If the Gui is passive, we subscribe to the SetTaskCommand Event and show
  // the incoming command in the Gui. We do the subscription here, since the
  // setPassive() method is called possibly after construction. Therefore we
  // can't decide the subscription upon construction time. This might lead to
  // issues if the start method is called several times, since the subscriber
  // will be re-created every time.
  if (this->passive)
  {
    subscribe<const MatNd*, const MatNd*>("SetTaskCommand",
                                          &TaskGuiComponent::onTaskCommand);
    this->gui = new Rcs::ControllerGui(&this->controller, this->a_des, this->x_des, this->x_des,
                                       this->x_des, &this->mtx, this->passive);
    subscribe("ChangeTaskVector", &TaskGuiComponent::onTaskVectorChange);
  }
  // If the Gui is active, we subscribe to a Gui callback that copies the
  // Slider values into the desired task coordinates.
  else
  {
    this->gui = new Rcs::ControllerGui(&this->controller, this->a_des, this->x_des,
                                       this->x_curr, &this->mtx, this->passive);
    Rcs::ControllerWidgetBase* w = static_cast<Rcs::ControllerWidgetBase*>(gui->getWidget());
    RCHECK(w);
    w->registerCallback(new TaskUpdateCallback(this));
  }

  RLOG(0, "Start::start()");
}

void TaskGuiComponent::stop()
{
  delete this->gui;
  this->gui = NULL;
}

void TaskGuiComponent::guiCallback()
{
  RLOG(5, "TaskGuiComponent::guiCallback()");
  pthread_mutex_lock(&this->mtx);
  filt->setTarget(this->x_des->ele);
  pthread_mutex_unlock(&this->mtx);
}

/*******************************************************************************
 * This is only subscribed if the Gui is passive. We then write x_des and a_des
 * into the internal arrays so that the Guimakes them visible.
 ******************************************************************************/
void TaskGuiComponent::onTaskCommand(const MatNd* a, const MatNd* x)
{
  RLOG(5, "SetTaskCommand::setTaskCommand(MatNd*,MatNd*)");
  if (this->passive)
  {
    pthread_mutex_lock(&this->mtx);
    MatNd_copy(this->a_des, a);
    MatNd_copy(this->x_des, x);
    pthread_mutex_unlock(&this->mtx);
  }
}

const MatNd* TaskGuiComponent::getActivationPtr() const
{
  return this->a_des;
}

const MatNd* TaskGuiComponent::getTaskCommandPtr() const
{
  return this->x_des_filt;
}

void TaskGuiComponent::setState(RcsGraph* from)
{
  RLOG(5, "ComputeKinematics::setState()");
  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(controller.getGraph(), from->q, NULL);
  filt->iterate();
  filt->getPosition(x_des_filt->ele);
  controller.computeX(this->x_curr);
  pthread_mutex_unlock(&this->mtx);
}

void TaskGuiComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "TaskGuiComponent::onInitFromState()");

  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(controller.getGraph(), target->q, target->q_dot);

  MatNd* x_target = MatNd_create(controller.getTaskDim(), 1);
  controller.computeX(x_target);

  MatNd_setZero(this->a_des);
  MatNd_copy(this->x_des, x_target);
  MatNd_copy(this->x_des_filt, x_target);
  MatNd_copy(this->x_curr, x_target);

  filt->init(x_target->ele);
  pthread_mutex_unlock(&this->mtx);

  if (gui)
  {
    Rcs::ControllerWidgetBase* widget = static_cast<Rcs::ControllerWidgetBase*>(gui->getWidget());
    widget->reset(this->a_des, x_target);
  }

  MatNd_destroy(x_target);
}

void TaskGuiComponent::onTaskVectorChange(std::vector<std::string> taskVec)
{
  if (!passive)
  {
    return;
  }

  RLOG(0, "Changing task vector");

  std::vector<Rcs::Task*> tasks;

  for (auto& t : taskVec)
  {
    Rcs::Task* ti = Rcs::TaskFactory::createTask(t, controller.getGraph());
    bool success = true;

    // We assume the cretion of a task as failure if:
    // - the TaskFactory returns NULL
    // - the returned task is already part of the task vector.
    if (!ti)
    {
      RLOG_CPP(0, "This task could not be created:\n\n" << t);
      success = false;
    }
    // else
    // {
    //   for (size_t j = 0; j < tc->getInternalController()->getNumberOfTasks(); ++j)
    //   {
    //     if (tc->getInternalController()->getTaskName(j) == ti->getName())
    //     {
    //       RLOG_CPP(0, "Task with same name already exists: " << ti->getName());
    //       success = false;
    //     }
    //   }

    // }

    // Check if task can be created. In case of failure, we return early.
    if (!success)
    {
      for (auto ti : tasks)
      {
        delete ti;
      }
      return;
    }
    else
    {
      tasks.push_back(ti);
    }
  }

  delete this->gui;

  controller.eraseTasks();

  for (auto t : tasks)
  {
    controller.add(t);
  }


  // From here on, all tasks are valid.
  MatNd_realloc(a_des, controller.getNumberOfTasks(), 1);
  MatNd_realloc(x_des, controller.getTaskDim(), 1);
  MatNd_realloc(x_des_filt, controller.getTaskDim(), 1);
  MatNd_realloc(x_curr, controller.getTaskDim(), 1);

  this->gui = new Rcs::ControllerGui(&this->controller, this->a_des, this->x_des, this->x_des,
                                     this->x_des, &this->mtx, this->passive);


  RLOG(0, "Done task replacement");
}

}   // namespace aff
