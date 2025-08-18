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

#include "ImageHelpers.h"
#include "SceneHelpers.h"
#include "SceneJsonHelpers.h"

#include <Rcs_macros.h>
#include <GraphNode.h>

#include <QByteArray>
#include <QLabel>
#include <QPixmap>
#include <QDebug>
#include <QThread>
#include <QApplication>
#include <QPointer>
#include <QMetaObject>
#include <QBuffer>
#include <QRect>

#include <algorithm>


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
PinholeCamera::PinholeCamera() : fx(0.0), fy(0.0), cx(0.0), cy(0.0)
{
}


/*******************************************************************************
 * Converts base64 JPEG to QImage
 ******************************************************************************/
QImage decodeBase64JpegToQImage(const QString& base64String)
{
  // Decode base64 to raw JPEG bytes
  QByteArray jpegData = QByteArray::fromBase64(base64String.toUtf8());

  // Decode JPEG bytes to QImage
  QImage image;
  if (!image.loadFromData(jpegData, "JPG"))
  {
    RLOG_CPP(1, "Failed to decode JPEG image");
    return QImage(); // returns a null image
  }

  return image;
}

/*******************************************************************************
 * Image gui singleton
 ******************************************************************************/
void showFrame(const QString& key, const QImage& img)
{
  if (img.isNull())
  {
    return;
  }

  // hop to GUI thread if needed
  if (QThread::currentThread() != qApp->thread())
  {
    QMetaObject::invokeMethod(
      qApp,
      [key, img] { showFrame(key, img); },
      Qt::QueuedConnection);
    return;
  }

  using WindowPtr = QPointer<QLabel>;
  static QHash<QString, WindowPtr> windows;

  // one-time app-shutdown cleanup while event loop still exists
  static bool cleanupHooked = false;
  if (!cleanupHooked)
  {
    cleanupHooked = true;
    QObject::connect(qApp, &QCoreApplication::aboutToQuit, []
    {
      for (auto& w : windows)
      {
        if (w)
        {
          w->deleteLater();
        }
      }
      windows.clear();
    });
  }

  // fetch or create window for this key
  WindowPtr& ref = windows[key];
  QLabel* w = ref;

  if (!w)
  {
    w = new QLabel;
    ref = w;

    w->setWindowTitle(key.isEmpty() ? QStringLiteral("Video Stream") : key);
    w->setAttribute(Qt::WA_DeleteOnClose, true);     // delete when user closes
    w->setAlignment(Qt::AlignCenter);
    w->setScaledContents(false);                     // keep pixel-perfect
    w->resize(img.size());

    // drop mapping when destroyed
    QObject::connect(w, &QObject::destroyed, [key]
    {
      // note: QPointer auto-nulls, we also remove the key entirely
      windows.remove(key);
    });

    w->show();
  }

  // update image
  if (w->isHidden())
  {
    w->show();  // revive if user closed/hid and we recreated
  }
  w->setPixmap(QPixmap::fromImage(img));
  if (w->size() != img.size())
  {
    w->resize(img.size());
  }
}

// Optional convenience overload for single-window use:
void showFrame(const QImage& img)
{
  showFrame(QStringLiteral("Video Stream"), img);
}


void showFrame_old(const QImage& img)
{
  if (img.isNull())
  {
    return;
  }

  // -- Always run UI work on the GUI thread ---------------------------
  if (QThread::currentThread() != qApp->thread())
  {
    QMetaObject::invokeMethod(
      qApp,
      [img] { showFrame(img); },
      Qt::QueuedConnection);
    return;
  }

  // -- Static members for window and cleanup --------------------------
  static QPointer<QLabel> window;
  static QLabel* rawWindow = nullptr;

  // RAII "guardian" whose destructor runs at program exit;
  // The Cleaner object is created the first time showFrame() runs and lives until static
  // destruction starts.Because its destructor only calls deleteLater(), no GUI code
  // executes if QApplication is already half - gone; Qt will simply enqueue the deletion
  // if the event loop is still running, or fall back to direct deletion otherwise.Static
  // locals are destroyed in reverse order of construction, guaranteeing window is still
  // valid when Cleaner runs.
  static struct Cleaner
  {
    ~Cleaner()
    {
      if (rawWindow)
      {
        rawWindow->deleteLater();
      }
    }
  } guard;

  // -- Create window on first use -------------------------------------
  if (window.isNull())
  {
    QLabel* w = new QLabel;
    w->setWindowTitle(QStringLiteral("Video Stream"));
    w->setAttribute(Qt::WA_DeleteOnClose, false);
    w->setAlignment(Qt::AlignCenter);
    w->setScaledContents(false); // keep aspect ratio
    w->resize(img.width(), img.height());

    // Track destruction to clear pointers
    QObject::connect(w, &QObject::destroyed, []
    {
      window.clear();
      rawWindow = nullptr;
    });

    window = w;
    rawWindow = w;

    w->show();
  }

  // -- Update frame ---------------------------------------------------
  if (window)
  {
    window->setPixmap(QPixmap::fromImage(img));
    window->resize(img.size());
  }
}


/*******************************************************************************
 * encryption
 ******************************************************************************/
std::string rgbToJpegBase64(const uint8_t* rgb,
                            int width,
                            int height,
                            int quality)          // 0-100
{
  if (!rgb || width <= 0 || height <= 0)
  {
    throw std::invalid_argument("rgbToJpegBase64: null pointer or invalid size");
  }

  // Wrap raw RGB into QImage (no copy yet)
  const int bytesPerLine = width * 3;
  QImage img(rgb, width, height, bytesPerLine, QImage::Format_RGB888);

  // Make an owned copy in case the source buffer goes away
  img = img.copy();

  // JPEG-encode into QByteArray
  QByteArray jpegBytes;
  {
    QBuffer buffer(&jpegBytes);
    buffer.open(QIODevice::WriteOnly);
    if (!img.save(&buffer, "JPEG", quality))
    {
      throw std::runtime_error("JPEG encoding failed");
    }
  }

  // Base-64 + data-URI
  QByteArray b64 = jpegBytes.toBase64();
  //QByteArray dataUri = "data:image/jpeg;base64," + b64;

  return { b64.constData(), static_cast<size_t>(b64.size()) };
}



/*******************************************************************************
 * Camera model from header json
 ******************************************************************************/
bool extract_intrinsics(const nlohmann::json& data,
                        double& fx, double& fy, double& cx, double& cy,
                        std::string& err)
{
  try
  {
    auto it = data.find("camera_matrix");
    if (it == data.end())
    {
      err = "Missing key: camera_matrix";
      return false;
    }
    const auto& cm = *it;
    if (!cm.is_array() || cm.size() != 3)
    {
      err = "camera_matrix must be a 3x3 array (outer size 3).";
      return false;
    }
    for (size_t r = 0; r < 3; ++r)
    {
      if (!cm[r].is_array() || cm[r].size() != 3)
      {
        err = "camera_matrix must be a 3x3 array (row size 3).";
        return false;
      }
    }

    fx = cm[0][0].get<double>();
    fy = cm[1][1].get<double>();
    cx = cm[0][2].get<double>();
    cy = cm[1][2].get<double>();
    return true;
  }
  catch (const std::exception& e)
  {
    err = std::string("JSON error: ") + e.what();
    return false;
  }
  catch (...)
  {
    err = "Unknown error while parsing intrinsics.";
    return false;
  }
}


}   // namespace

