#include "GenericGattMonitor.h"
#include <BLERemoteService.h>

namespace bcb {

int GenericGattMonitor::subscribeToAll(BLEClient* client) {
  int subscribedCount = 0;
  if (client == nullptr || !client->isConnected()) {
    return 0;
  }

  std::map<std::string, BLERemoteService*>* services = client->getServices();
  if (services == nullptr) {
    return 0;
  }

  for (auto& svcEntry : *services) {
    BLERemoteService* svc = svcEntry.second;
    std::map<std::string, BLERemoteCharacteristic*>* chars = svc->getCharacteristics();
    if (chars == nullptr) continue;

    for (auto& chrEntry : *chars) {
      BLERemoteCharacteristic* chr = chrEntry.second;

      GattCharacteristicInfo info;
      info.serviceUUID = std::string(svc->getUUID().toString().c_str());
      info.characteristicUUID = std::string(chr->getUUID().toString().c_str());
      info.canRead = chr->canRead();
      info.canWrite = chr->canWrite();
      info.canNotify = chr->canNotify();
      info.canIndicate = chr->canIndicate();

      if (_onEnumerated) {
        _onEnumerated(info);
      }

      if (!chr->canNotify() && !chr->canIndicate()) {
        continue;
      }

      std::string svcUuidStr = info.serviceUUID;
      std::string chrUuidStr = info.characteristicUUID;

      // Captured by value (svcUuidStr/chrUuidStr/this) so the callback
      // remains valid after this loop iteration ends -- registerForNotify
      // stores the std::function for the lifetime of the characteristic.
      auto handler = [this, svcUuidStr, chrUuidStr](
                         BLERemoteCharacteristic* /*chr*/, uint8_t* data, size_t length, bool /*isNotify*/) {
        if (_onNotification) {
          GattNotification evt;
          evt.data = data;
          evt.length = length;
          evt.serviceUUID = svcUuidStr;
          evt.characteristicUUID = chrUuidStr;
          _onNotification(evt);
        }
      };

      chr->registerForNotify(handler, /*notifications=*/chr->canNotify());
      subscribedCount++;
    }
  }

  return subscribedCount;
}

}  // namespace bcb
