#pragma once

#include "VAOMesh.h"

enum cloth_type { SINGLE_LAYER_NOB, SINGLE_LAYER_BOUNDARY };
class Cloth : public VAOMesh
{
public:
	Cloth(cloth_type type = SINGLE_LAYER_BOUNDARY) : _type(type) { }
	cloth_type get_obj_type() const { return _type;  }
private:
	cloth_type _type;
};


