#include "quat_math.h"

float sqrt_f(float x)
{
	if(x < 0) return -1;
	if(x == 0) return 0;
	float aa = x;
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
	for(int nn = 0; nn < 6; ++nn)
	{
		float avg = 0.5f*(aa+bb);
		bb *= aa/avg;
		aa = avg;
	}  
	return mult*0.5f*(aa+bb);  
}
float sin_f(float v)
{
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

float atan2_f(float y, float x)
{
	float ax = x;
	float ay = y;
	if(ax < 0) ax = -x;
	if(ay < 0) ay = -y;
	float mn = ax;
	float mx = ax;
	if(mx < 0.0000001f) return 0; //0,0 - undefined
	if(ay > ax) mx = ay;
	else mn = ay;
	float a = mn / mx;
	float s = a*a;
	float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
	if(ay > ax) r = 1.57079637f - r;
	if(x < 0) r = 3.1415926f - r;
	if(y < 0) r = -r;
	return r;
}

float asin_f(float x)
{
	float sgn = 1;
	if(x < 0) sgn = -1, x = -x;
	
	float x21 = x*2.0f - 1.0f;
	float x21_2 = x21*x21;
	float x21_3 = x21*x21_2;
	float x21_4 = x21*x21_3;
	float x21_5 = x21*x21_4;
	float x21_6 = x21*x21_5;
	float res = -0.09902914f + 1.235307f*x + .2237106f*x21_2 - .1746904f * x21_3 - .42491176f * x21_4 + .30870647f * x21_5 + .4401081f * x21_6;
	return sgn * res;
}

float acos_f(float x) 
{
	return 3.1415926*0.5 - asin_f(x);
}

float q_norm(sQ *q)
{
	return sqrt_f(q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w);
}

float v_norm(sV *v)
{
	return sqrt_f(v->x*v->x + v->y*v->y + v->z*v->z);
}

void q_renorm(sQ *q)
{
	float r = q_norm(q);
	if(r > 0)
	{
		float m = 1.0f / r;
		q->x *= m;	
		q->y *= m;	
		q->z *= m;	
		q->w *= m;	
	}
}

void v_renorm(sV *v)
{
	float r = v_norm(v);
	if(r > 0)
	{
		float m = 1.0f / r;
		v->x *= m;	
		v->y *= m;	
		v->z *= m;	
	}
}

void q_make_conj(sQ *q, sQ *qc)
{
	qc->x = -q->x;
	qc->y = -q->y;
	qc->z = -q->z;
	qc->w = q->w;
}

void q_set(sQ *target, sQ *source)
{
	target->x = source->x;
	target->y = source->y;
	target->z = source->z;
	target->w = source->w;
}

void v_set(sV *target, sV *source)
{
  target->x = source->x;
  target->y = source->y;
  target->z = source->z;
}

void q_mult(sQ *q1, sQ *q2, sQ *qr)
{
// x1  y1  z1
// x2  y2  z2
// x   y   z
	qr->w = q1->w*q2->w - (q1->x*q2->x + q1->y*q2->y + q1->z*q2->z);
	qr->x = q1->w*q2->x + q2->w*q1->x + q1->y*q2->z - q1->z*q2->y;
	qr->y = q1->w*q2->y + q2->w*q1->y + q1->z*q2->x - q1->x*q2->z;
	qr->z = q1->w*q2->z + q2->w*q1->z + q1->x*q2->y - q1->y*q2->x;
}

void rotate_v(sQ *q, sV *v)
{
	sQ r;
	r.w = 0;
	r.x = v->x;
	r.y = v->y;
	r.z = v->z;
	sQ qc, qq, rq;
	q_make_conj(q, &qc);
	q_mult(&r, &qc, &qq);
	q_mult(q, &qq, &rq);
	v->x = rq.x;
	v->y = rq.y;
	v->z = rq.z;
}

void qv_mult(sQ *q1, sQ *q2, sQ *res)
{
// x1  y1  z1
// x2  y2  z2
// x   y   z
	res->w = 0;
	res->x = q1->y*q2->z - q1->z*q2->y;
	res->y = q1->z*q2->x - q1->x*q2->z;
	res->z = q1->x*q2->y - q1->y*q2->x;
}

void v_mult(sV *v1, sV *v2, sV *res)
{
// x1  y1  z1
// x2  y2  z2
// x   y   z
	res->x = v1->y*v2->z - v1->z*v2->y;
	res->y = v1->z*v2->x - v1->x*v2->z;
	res->z = v1->x*v2->y - v1->y*v2->x;
}
float v_dot(sV *v1, sV *v2)
{
	return v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;
}

void q_from_vectors(sV *u, sV *v, sQ *res) 
{
  float d = v_dot(u, v);
  sV w;
  v_mult(u, v, &w);
  res->w = d + sqrt_f(d*d + v_dot(&w, &w));
  res->x = w.x;
  res->y = w.y;
  res->z = w.z;

  q_renorm(res);
}