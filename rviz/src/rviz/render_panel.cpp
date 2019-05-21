/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QApplication>
#include <QMenu>
#include <QTimer>

#include <OgreSceneManager.h>
#include <OgreCamera.h>

#include "rviz/display.h"
#include "rviz/view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"
#include "rviz/window_manager_interface.h"

#include "rviz/render_panel.h"

namespace rviz
{

RenderPanel::RenderPanel( QWidget* parent )
  : QtOgreRenderWindow( parent )
  , mouse_x_( 0 )
  , mouse_y_( 0 )
  , focus_on_mouse_move_( true )
  , context_( 0 )
  , scene_manager_( 0 )
  , view_controller_( 0 )
  , context_menu_visible_(false)
  , fake_mouse_move_event_timer_( new QTimer() )
  , default_camera_(0)
{
  setFocusPolicy(Qt::WheelFocus);
  setFocus( Qt::OtherFocusReason );
  setAttribute(Qt::WA_AcceptTouchEvents);
  b_enable_touch_ = true;
  touch_distance_ = -1.0;
}

RenderPanel::~RenderPanel()
{
  delete fake_mouse_move_event_timer_;
  if( scene_manager_ && default_camera_ )
  {
    scene_manager_->destroyCamera( default_camera_ );
  }
  if( scene_manager_ )
  {
    scene_manager_->removeListener( this );
  }
}

void RenderPanel::initialize(Ogre::SceneManager* scene_manager, DisplayContext* context)
{
  context_ = context;
  scene_manager_ = scene_manager;
  scene_manager_->addListener( this );

  std::stringstream ss;
  static int count = 0;
  ss << "RenderPanelCamera" << count++;
  default_camera_ = scene_manager_->createCamera(ss.str());
  default_camera_->setNearClipDistance(0.01f);
  default_camera_->setPosition(0, 10, 15);
  default_camera_->lookAt(0, 0, 0);

  setCamera( default_camera_ );

  connect( fake_mouse_move_event_timer_, SIGNAL( timeout() ), this, SLOT( sendMouseMoveEvent() ));
  fake_mouse_move_event_timer_->start( 33 /*milliseconds*/ );
}


bool RenderPanel::event(QEvent *event)
{
    switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
    {
      QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
      QList<QTouchEvent::TouchPoint> touchPoints = touchEvent->touchPoints();
      int last_x = touch_mouse_x_
        , last_y = touch_mouse_y_;

      QEvent::Type ev;
      switch( event->type()){
        case QEvent::TouchBegin:
          ev = QEvent::MouseButtonPress;
          break;
        case QEvent::TouchUpdate:
          ev = QEvent::MouseMove;
          break;
        case QEvent::TouchEnd:
          ev = QEvent::MouseButtonRelease;
          break;
      }

      if( touchPoints.count() != 2 && touch_distance_>=0.0 ){
        touch_distance_ = -1.0;
        QMouseEvent e( QEvent::MouseButtonRelease
          , QPointF(mouse_x_, mouse_y_), Qt::MidButton, Qt::MidButton, Qt::NoModifier);
        onRenderWindowMouseEvents_touch( &e );
        printf("2 release\n");
      }

      if (touchPoints.count() == 1 ){
        QTouchEvent::TouchPoint touchPoint = touchEvent->touchPoints().first();
        
        QMouseEvent e( ev
          ,touchPoint.pos(), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        onRenderWindowMouseEvents_touch( &e );
      }
      else if (touchPoints.count() == 2) {
        const QTouchEvent::TouchPoint &touchPoint0 = touchPoints.first();
        const QTouchEvent::TouchPoint &touchPoint1 = touchPoints.last();
        double d1 = QLineF(touchPoint0.pos(), touchPoint1.pos()).length()
          , d2 = QLineF(touchPoint0.lastPos(), touchPoint1.lastPos()).length();

        QPointF centreZoom = QPointF((touchPoint0.pos().x()+ touchPoint1.pos().x())/2 ,
            (touchPoint0.pos().y()+ touchPoint1.pos().y())/2);
        QPointF lastCenterZoom = QPointF((touchPoint0.lastPos().x()+ touchPoint1.lastPos().x())/2 ,
            (touchPoint0.lastPos().y()+ touchPoint1.lastPos().y())/2);

        QString ev_name;
        if( touch_distance_<0.0 ){
          touch_distance_ = d1;
          mouse_x_ = centreZoom.x();
          mouse_y_ = centreZoom.y();
          printf("touch set mouse\n");
          QMouseEvent e( QEvent::MouseButtonPress, centreZoom, Qt::MidButton, Qt::MidButton, Qt::NoModifier);
          onRenderWindowMouseEvents_touch( &e );
          ev_name = "press";
        }
        else{
          QMouseEvent e( QEvent::MouseMove, centreZoom, Qt::NoButton, Qt::MidButton, Qt::NoModifier);
          onRenderWindowMouseEvents_touch( &e );
          ev_name = "move";

          const double DIS_DELTA = 0.1; // distance to count as 1 delta
          int delta = int ((d1 - touch_distance_) / DIS_DELTA);
          printf("%.4lf / %d\n", touch_distance_, delta);
          {
            touch_distance_+= delta * DIS_DELTA;
            QWheelEvent w( centreZoom // pos
              , delta // qt4Delta
              , Qt::NoButton, Qt::NoModifier
            );
            wheelEvent_touch( &w );
          }
        }
        printf("2 %s : %d, %d\n", ev_name.toStdString().c_str()
          , (int)centreZoom.x(), (int)centreZoom.y());

/*        if( touch_distance_ < 0.0 ){
          printf("touch 2 start\n");
          touch_distance_ = d1;
        }
        else{
          const double DIS_DELTA = 5.0; // distance to count as 1 delta
          int delta = int ((d1 - touch_distance_) / DIS_DELTA);
          printf("%.4lf / %d\n", touch_distance_, delta);
          {
            touch_distance_+= delta * DIS_DELTA;
            QWheelEvent w( centreZoom // pos
              , delta // qt4Delta
              , Qt::LeftButton, Qt::NoModifier
            );
            //wheelEvent_touch( &w );
          }
        }*/
      }
      break;
    }
    default:
        return QWidget::event(event);
    }
    return true;
}

void RenderPanel::sendMouseMoveEvent()
{
  QPoint cursor_pos = QCursor::pos();
  QPoint mouse_rel_widget = mapFromGlobal( cursor_pos );
  if( rect().contains( mouse_rel_widget ))
  {
    bool mouse_over_this = false;
    QWidget *w = QApplication::widgetAt( cursor_pos );
    while( w )
    {
      if( w == this )
      {
        mouse_over_this = true;
        break;
      }
      w = w->parentWidget();
    }
    if( !mouse_over_this )
    {
      return;
    }

    QMouseEvent fake_event( QEvent::MouseMove,
                            mouse_rel_widget,
                            Qt::NoButton,
                            QApplication::mouseButtons(),
                            QApplication::keyboardModifiers() );
    onRenderWindowMouseEvents( &fake_event );
  }
}
void RenderPanel::leaveEvent ( QEvent * event )
{
  setCursor( Qt::ArrowCursor );
  if ( context_ )
  {
    context_->setStatus("");
  }
}

void RenderPanel::onRenderWindowMouseEvents( QMouseEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_)
  {
    if (focus_on_mouse_move_) {
      setFocus( Qt::MouseFocusReason );
    }

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}


void RenderPanel::onRenderWindowMouseEvents_touch( QMouseEvent* event )
{
  int last_x = touch_mouse_x_;
  int last_y = touch_mouse_y_;

  touch_mouse_x_ = event->x();
  touch_mouse_y_ = event->y();

  printf("mid : %d, %d, %d, %d, %d, %d, %d, %d\n", (int)event->type(), (int)event->x(), (int)event->y()
    , (int)event->button(), (int)event->buttons(), (int)event->modifiers()
    , last_x, last_y );

  if (context_)
  {
    if (focus_on_mouse_move_) {
      setFocus( Qt::MouseFocusReason );
    }

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}



void RenderPanel::wheelEvent_touch( QWheelEvent* event )
{
  int last_x = touch_mouse_x_;
  int last_y = touch_mouse_y_;

  touch_mouse_x_ = event->x();
  touch_mouse_y_ = event->y();

  printf("whe2: %d, %d, %d, %d, %d, %d\n", (int)event->type(), (int)event->x(), (int)event->y()
    , (int)event->delta(), (int)event->buttons(), (int)event->modifiers() );

  if (context_)
  {
    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}



void RenderPanel::wheelEvent( QWheelEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  printf("whe : %d, %d, %d, %d, %d, %d\n", (int)event->type(), (int)event->x(), (int)event->y()
    , (int)event->delta(), (int)event->buttons(), (int)event->modifiers() );

  if (context_)
  {
    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::keyPressEvent( QKeyEvent* event )
{
  if( context_ )
  {
    context_->handleChar( event, this );
  }
}

void RenderPanel::setViewController( ViewController* controller )
{
  view_controller_ = controller;

  if( view_controller_ )
  {
    setCamera( view_controller_->getCamera() );
    view_controller_->activate();
  }
  else
  {
    setCamera( NULL );
  }
}

void RenderPanel::showContextMenu( boost::shared_ptr<QMenu> menu )
{
  boost::mutex::scoped_lock lock(context_menu_mutex_);
  context_menu_ = menu;
  context_menu_visible_ = true;

  QApplication::postEvent( this, new QContextMenuEvent( QContextMenuEvent::Mouse, QPoint() ));
}

void RenderPanel::onContextMenuHide()
{
  context_menu_visible_ = false;
}

bool RenderPanel::contextMenuVisible()
{
  return context_menu_visible_;
}

void RenderPanel::contextMenuEvent( QContextMenuEvent* event )
{
  boost::shared_ptr<QMenu> context_menu;
  {
    boost::mutex::scoped_lock lock(context_menu_mutex_);
    context_menu.swap(context_menu_);
  }

  if ( context_menu )
  {
    connect( context_menu.get(), SIGNAL( aboutToHide() ), this, SLOT( onContextMenuHide() ) );
    context_menu->exec( QCursor::pos() );
  }
}

void RenderPanel::sceneManagerDestroyed( Ogre::SceneManager* destroyed_scene_manager )
{
  if( destroyed_scene_manager == scene_manager_ )
  {
    scene_manager_ = NULL;
    default_camera_ = NULL;
    setCamera( NULL );
  }
}

bool RenderPanel::getFocusOnMouseMove() const
{
  return focus_on_mouse_move_;
}

void RenderPanel::setFocusOnMouseMove(bool enabled)
{
  focus_on_mouse_move_ = enabled;
}

} // namespace rviz
