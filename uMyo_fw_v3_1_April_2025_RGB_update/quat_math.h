
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


float q_norm(sQ q);
float v_norm(sV v);
void q_renorm(sQ *q);
void v_renorm(sV *v);
sQ q_make_conj(sQ q);
sQ q_mult(sQ q1, sQ q2);
sV rotate_v(sQ q, sV v);
sQ qv_mult(sQ q1, sQ q2);
sV v_mult(sV v1, sV v2);
float v_dot(sV v1, sV v2);
sQ q_from_vectors(sV u, sV v);

float sin_f(float v);
float cos_f(float v);
float asin_f(float x);
float acos_f(float x);
float atan2_f(float y, float x);
