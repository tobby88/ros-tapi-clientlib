#ifndef FEATURE_H
#define FEATURE_H

#include <string>

namespace Tapi
{
class Feature
{
public:
  // Constructor/Destructor
  Feature(std::string type, std::string name, std::string uuid);
  ~Feature();

  // Public member functions
  std::string GetName() const;
  std::string GetType() const;
  std::string GetUUID() const;
  bool operator==(const Feature &other) const;
  void Update(std::string type, std::string name);

private:
  // Private member variables
  std::string type;
  std::string name;
  std::string uuid;
};
}

#endif  // FEATURE_H
