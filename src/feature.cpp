#include "include/tapi_lib/feature.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Feature::Feature(string type, string name, string uuid) : type(type), name(name), uuid(uuid)
{
}

Feature::~Feature()
{
}

// Public member functions

string Feature::GetName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

string Feature::GetType() const
{
  return type;
}

string Feature::GetUUID() const
{
  return uuid;
}

bool Feature::operator==(const Feature &other) const
{
  if (GetUUID() == other.GetUUID() && GetType() == other.GetType() && GetName() == other.GetName())
    return true;
  else
    return false;
}

void Feature::Update(string type, std::string name)
{
  this->type = type;
  this->name = name;
}
}
