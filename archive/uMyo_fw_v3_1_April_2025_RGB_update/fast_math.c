
float sin_f(float v)
{
/*********************************************************
 * high precision sine/cosine
 *********************************************************/
	float res = 0;
	while (v < -3.14159265f) v += 6.28318531f;
	while(v >  3.14159265f) v -= 6.28318531f;

	if (v < 0)
	{
		res = 1.27323954f * v + .405284735f * v * v;

		if (res < 0)
			res = .225f * (res *-res - res) + res;
		else
			res = .225f * (res * res - res) + res;
	}
	else
	{
		res = 1.27323954f * v - 0.405284735f * v * v;

		if (res < 0)
			res = .225f * (res *-res - res) + res;
		else
			res = .225f * (res * res - res) + res;
	}
	return res;
}

float cos_f(float v)
{
	return sin_f(v+1.57079632f);
}

float sin_small_f(float v) //precise for small v only
{
	return v - 0.1666667f*v*v*v;
}
float cos_small_f(float v) //precise for small v only
{
	return 1.0f - 0.5f*v*v + 0.0416667f*v*v*v*v;
}
