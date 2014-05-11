#ifndef _KALMAN_H
#define _KALMAN_H

//#define KALMAN_TYPE float

template <typename KALMAN_TYPE>
class _KALMAN
{
  public:
	_KALMAN(){
	  // random default = obviously there must be a better way?
		Sz = 1; // R
		Sw = 1; // Q
		x = 0;
		x_last = 0;
		P = 0;
		P_last = 0;
	}

	_KALMAN(KALMAN_TYPE R, KALMAN_TYPE Q){
	  // random default = obviously there must be a better way?
		Sz = R;
		Sw = Q;

		x = 0;
		x_last = 0;
		P = 0;
		P_last = 0;
	}


	~_KALMAN(){}  

//
	void update(KALMAN_TYPE measurement)
	{
	  KALMAN_TYPE P_temp, K, x_temp;

	  //predict
	  x_temp = x_last;
	  P_temp = P_last + Sw; //Q

	  //update
	  K = (1/(P_temp + Sz)) * P_temp; //R
	  x = x_temp + K * (measurement - x_temp);
	  P = (1 - K) * P_temp;

	  //save previous states
	  x_last = x;
	  P_last = P;
	}

	KALMAN_TYPE getX(){return x;}
	KALMAN_TYPE getP(){return P;}

  private:
  KALMAN_TYPE x;
  KALMAN_TYPE x_last;
  KALMAN_TYPE P;
  KALMAN_TYPE P_last;
  KALMAN_TYPE Sz;
  KALMAN_TYPE Sw;
};

typedef _KALMAN<float> KALMAN;
//typedef _KALMAN<double> KALMAN;

#endif
