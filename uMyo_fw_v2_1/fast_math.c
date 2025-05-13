
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


float sqrt_f(float v)
{
	if(v < 0) return -1;
	if(v == 0) return 0;
	float aa = v;
	float bb = 1;
	float mult = 1;
	int m_iters = 0;
	while(aa > 100 && m_iters++ < 20)
	{
		aa *= 0.01;
		mult *= 10;
	}
	while(aa < 0.01 && m_iters++ < 20)
	{
		aa *= 100;
		mult *= 0.1;
	}
	for(int nn = 0; nn < 8; ++nn)
	{
		float avg = 0.5*(aa+bb);
		bb *= aa/avg;
		aa = avg;
	}  
	return mult*0.5*(aa+bb);	
}