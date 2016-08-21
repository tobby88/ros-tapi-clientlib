#ifndef APIGUI_HPP
#define APIGUI_HPP

#include "api.hpp"
#include "device.hpp"
#include "guidevice.hpp"
#include <QPoint>
#include <QTimer>
#include <QVBoxLayout>
#include <vector>

using namespace std;

namespace Ui
{
class ApiGui;
}

class ApiGui : public QWidget
{
  Q_OBJECT

public:
  // Public member functions
  ApiGui(Api* api, QWidget* parent = 0);
  ~ApiGui();

protected:
  // Protected member functions
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  Api* api;
  QVBoxLayout* layoutReceiver;
  QVBoxLayout* layoutSender;
  QPoint mousePosition;
  vector<GuiDevice*> receiverGuiDevices;
  Feature* selectedFeature;
  GuiDevice* selectedGuiDevice;
  vector<GuiDevice*> senderGuiDevices;
  unsigned int temp2;
  QTimer* timer;
  int timerInterval;
  Ui::ApiGui* ui;

  // Private member functions
  void addDevice(Device* device);

private slots:
  // Slot functions
  void checkApiForUpdate();
  void featureClicked(GuiDevice* guidevice, Feature* feature);
};

#endif // APIGUI_HPP
