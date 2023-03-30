#pragma once
#include "data_description.h"

namespace operations
{
	void quaternion2euler(const data_description::quaternion&, data_description::euler&);
}