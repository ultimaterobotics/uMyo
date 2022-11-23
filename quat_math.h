
#define M_PI 3.1415926

typedef struct sQ
{
	float x;
	float y;
	float z;
	float w;
}sQ;

typedef struct sV
{
	float x;
	float y;
	float z;
}sV;


float q_norm(sQ *q);
float v_norm(sV *v);
void q_renorm(sQ *q);
void v_renorm(sV *v);
void q_make_conj(sQ *q, sQ *qc);
void q_mult(sQ *q1, sQ *q2, sQ *qr);
void q_set(sQ *target, sQ *source);
void rotate_v(sQ *q, sV *v);
void qv_mult(sQ *q1, sQ *q2, sQ *res);
void v_mult(sV *v1, sV *v2, sV *res);
float v_dot(sV *v1, sV *v2);
void v_set(sV *target, sV *source);
void q_from_vectors(sV *u, sV *v, sQ *res);

float sqrt_f(float x);
float sin_f(float v);
float cos_f(float v);
float asin_f(float x);
float atan2_f(float y, float x);