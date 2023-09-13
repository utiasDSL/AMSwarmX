#include "amswarmx/trajectory_utils.hpp"

double binomialCoeff(double n, double k)
{
	if (k == 0 || k == n)
		return 1;

	return binomialCoeff(n - 1, k - 1) +
		binomialCoeff(n - 1, k);
}
Eigen :: ArrayXXd diff(Eigen :: ArrayXXd arr)
{
	Eigen :: ArrayXXd temp(arr.rows() - 1, 1);
	for (int i = 0; i < arr.rows() - 1; i++)
	{
	temp(i) = arr(i + 1) - arr(i);
	}
	return temp;
}
Eigen :: ArrayXXd arctan2(Eigen :: ArrayXXd arr1, Eigen :: ArrayXXd arr2)
{
	Eigen :: ArrayXXd temp(arr1.rows(), arr1.cols());

	int k = 0;
	for (int i = 0; i < arr1.cols(); i++)
	{
		for (int j = 0; j < arr1.rows(); j++)
		{
			temp(k) = atan2(arr1(k), arr2(k));
			k++;
		}
	}
	return temp;
}
Eigen :: ArrayXXd reshape(Eigen :: ArrayXXd x, uint32_t r, uint32_t c)
{
	// starts with columns 
	Eigen :: Map<Eigen :: ArrayXXd> rx(x.data(), r, c);
	return rx;
}
Eigen :: ArrayXXd stack(Eigen :: ArrayXXd arr1, Eigen :: ArrayXXd arr2, char ch)
{
	if (ch == 'v')
	{
		Eigen :: ArrayXXd temp(arr1.rows() + arr2.rows(), arr1.cols());
		temp << arr1, arr2;
		return temp;
	}
	else
	{
		Eigen :: ArrayXXd temp(arr1.rows(), arr1.cols() + arr2.cols());
		temp << arr1, arr2;
		return temp;
	}

}
five_var bernsteinCoeff(double n, double tmin, double tmax, Eigen :: ArrayXXd t_actual, int num)
{
	five_var s;
	double l = tmax - tmin;
	Eigen :: ArrayXXd t = (t_actual - tmin) / l;

	Eigen :: ArrayXXd P(num, (int)n + 1), Pdot(num, (int)n + 1), Pddot(num, (int)n + 1), Pdddot(num, (int)n + 1), Pddddot(num, (int)n + 1);

	if((int)n == 10){
		P.col(0) = binomialCoeff(n, 0) * pow(1 - t, n - 0) * pow(t, 0);
		P.col(1) = binomialCoeff(n, 1) * pow(1 - t, n - 1) * pow(t, 1);
		P.col(2) = binomialCoeff(n, 2) * pow(1 - t, n - 2) * pow(t, 2);
		P.col(3) = binomialCoeff(n, 3) * pow(1 - t, n - 3) * pow(t, 3);
		P.col(4) = binomialCoeff(n, 4) * pow(1 - t, n - 4) * pow(t, 4);
		P.col(5) = binomialCoeff(n, 5) * pow(1 - t, n - 5) * pow(t, 5);
		P.col(6) = binomialCoeff(n, 6) * pow(1 - t, n - 6) * pow(t, 6);
		P.col(7) = binomialCoeff(n, 7) * pow(1 - t, n - 7) * pow(t, 7);
		P.col(8) = binomialCoeff(n, 8) * pow(1 - t, n - 8) * pow(t, 8);
		P.col(9) = binomialCoeff(n, 9) * pow(1 - t, n - 9) * pow(t, 9);
		P.col(10) = binomialCoeff(n, 10) * pow(1 - t, n - 10) * pow(t, 10);

		Pdot.col(0) = -10.0 * pow(-t + 1, 9);
		Pdot.col(1) = -90.0 * t * pow(-t + 1, 8) + 10.0 * pow(-t + 1, 9);
		Pdot.col(2) = -360.0 * pow(t, 2) * pow(-t + 1, 7) + 90.0 * t * pow(-t + 1, 8);
		Pdot.col(3) = -840.0 * pow(t, 3) * pow(-t + 1, 6) + 360.0 * pow(t, 2) * pow(-t + 1, 7);
		Pdot.col(4) = -1260.0 * pow(t, 4) * pow(-t + 1, 5) + 840.0 * pow(t, 3) * pow(-t + 1, 6);
		Pdot.col(5) = -1260.0 * pow(t, 5) * pow(-t + 1, 4) + 1260.0 * pow(t, 4) * pow(-t + 1, 5);
		Pdot.col(6) = -840.0 * pow(t, 6) * pow(-t + 1, 3) + 1260.0 * pow(t, 5) * pow(-t + 1, 4);
		Pdot.col(7) = -360.0 * pow(t, 7) * pow(-t + 1, 2) + 840.0 * pow(t, 6) * pow(-t + 1, 3);
		Pdot.col(8) = 45.0 * pow(t, 8) * (2 * t - 2) + 360.0 * pow(t, 7) * pow(-t + 1, 2);
		Pdot.col(9) = -10.0 * pow(t, 9) + 9 * pow(t, 8) * (-10.0 * t + 10.0);
		Pdot.col(10) = 10.0 * pow(t, 9);

		Pddot.col(0) = 90.0 * pow(-t + 1, 8.0);
		Pddot.col(1) = 720.0 * t * pow(-t + 1, 7) - 180.0 * pow(-t + 1, 8);
		Pddot.col(2) = 2520.0 * pow(t, 2) * pow(-t + 1, 6) - 1440.0 * t * pow(-t + 1, 7) + 90.0 * pow(-t + 1, 8);
		Pddot.col(3) = 5040.0 * pow(t, 3) * pow(-t + 1, 5) - 5040.0 * pow(t, 2) * pow(-t + 1, 6) + 720.0 * t * pow(-t + 1, 7);
		Pddot.col(4) = 6300.0 * pow(t, 4) * pow(-t + 1, 4) - 10080.0 * pow(t, 3) * pow(-t + 1, 5) + 2520.0 * pow(t, 2) * pow(-t + 1, 6);
		Pddot.col(5) = 5040.0 * pow(t, 5) * pow(-t + 1, 3) - 12600.0 * pow(t, 4) * pow(-t + 1, 4) + 5040.0 * pow(t, 3) * pow(-t + 1, 5);
		Pddot.col(6) = 2520.0 * pow(t, 6) * pow(-t + 1, 2) - 10080.0 * pow(t, 5) * pow(-t + 1, 3) + 6300.0 * pow(t, 4) * pow(-t + 1, 4);
		Pddot.col(7) = -360.0 * pow(t, 7) * (2 * t - 2) - 5040.0 * pow(t, 6) * pow(-t + 1, 2) + 5040.0 * pow(t, 5) * pow(-t + 1, 3);
		Pddot.col(8) = 90.0 * pow(t, 8) + 720.0 * pow(t, 7) * (2 * t - 2) + 2520.0 * pow(t, 6) * pow(-t + 1, 2);
		Pddot.col(9) = -180.0 * pow(t, 8) + 72 * pow(t, 7) * (-10.0 * t + 10.0);
		Pddot.col(10) = 90.0 * pow(t, 8);

		Pdddot.col(0) = -720.0 * pow(-t + 1, 7);
		Pdddot.col(1) = 2160 * pow(1 - t, 7) - 5040 * pow(1 - t, 6) * t;
		Pdddot.col(2) = -15120 * pow(1-t, 5) * pow(t, 2) + 15120 * pow(1 - t, 6) * t - 2160 * pow(1 - t, 7);
		Pdddot.col(3) = -720 * pow(t-1, 4) * (120 * pow(t, 3) - 108 * pow(t, 2) + 24 * t - 1);
		Pdddot.col(4) = 5040 * pow(t-1, 3) * t * (30 * pow(t, 3) - 36*pow(t, 2) + 12 * t - 1);
		Pdddot.col(5) = -15120 * pow(t-1, 2) * pow(t, 2) * (12 * pow(t, 3) - 18*pow(t, 2) + 8 * t - 1);
		Pdddot.col(6) = 5040 * (t - 1) * pow(t, 3) * (30 * pow(t, 3)-54 * pow(t, 2) + 30 * t - 5);
		Pdddot.col(7) = -720 * pow(t, 7) + 10080 * (1-t) * pow(t, 6) - 45360 * pow(1 - t, 2) * pow(t, 5) + 25200 * pow(1-t, 3) * pow(t, 4) - 2520 * pow(t, 6) * (2 * t - 2);
		Pdddot.col(8) = 2160 * pow(t, 7) - 5040 * (1 - t) * pow(t, 6) + 15120 * pow(1-t, 2) * pow(t, 5) + 5040 * pow(t, 6) * (2 * t - 2);
		Pdddot.col(9) = 504 * (10 - 10 * t) * pow(t, 6) - 2160 * pow(t, 7);
		Pdddot.col(10) = 720.0 * pow(t, 7);

		Pddddot.col(0) = 5040.0 * pow(-t +1, 6);
		Pddddot.col(1) = -4320 * pow(t - 1, 5) * (10 * t - 3)-7200 * pow(t - 1, 6);
		Pddddot.col(2) = 10800 * pow(t - 1, 4) * (15 * pow(t, 2) - 9 * t + 1) + 2160 * pow(t-1, 5) * (30 * t - 9);
		Pddddot.col(3) = -20160 * pow(t-1, 3) * (30 * pow(t, 3) - 36 * pow(t, 2) + 12 * t - 1);
		Pddddot.col(4) = 5040 * pow(t-1, 2) * (210 * pow(t, 4) - 336 * pow(t, 3) + 168 * pow(t, 2) - 28 * t + 1);
		Pddddot.col(5) = -30240 * (t - 1) * t * (42 * pow(t, 4) - 84 * pow(t, 3) + 56 * pow(t, 2) - 14 * t + 1);
		Pddddot.col(6) = 1058400 * pow(t, 6) - 2540160 * pow(t, 5) + 2116800 * pow(t, 4) - 705600 * pow(t, 3) + 75600 * pow(t, 2);
		Pddddot.col(7) = -604800 * pow(t, 6) + 1088640 * pow(t, 5) - 604800 * pow(t, 4) + 100800 * pow(t, 3);
		Pddddot.col(8) = 226800 * pow(t, 6) - 272160 * pow(t, 5) + 75600 * pow(t, 4);
		Pddddot.col(9) = 30240 * pow(t, 5) - 50400 * pow(t, 6);
		Pddddot.col(10) = 5040.0 * pow(t, 6);
	}
	else if((int)n == 5){
		P.col(0) = binomialCoeff(n, 0) * pow(1 - t, 5) * pow(t, 0);
		P.col(1) = binomialCoeff(n, 1) * pow(1 - t, 4) * pow(t, 1);
		P.col(2) = binomialCoeff(n, 2) * pow(1 - t, 3) * pow(t, 2);
		P.col(3) = binomialCoeff(n, 3) * pow(1 - t, 2) * pow(t, 3);
		P.col(4) = binomialCoeff(n, 4) * pow(1 - t, 1) * pow(t, 4);
		P.col(5) = binomialCoeff(n, 5) * pow(1 - t, 0) * pow(t, 5);

		Pdot.col(0) = binomialCoeff(n, 0) * (-5*pow(t-1,4));
		Pdot.col(1) = binomialCoeff(n, 1) * (pow(t-1, 3) * (5*t-1));
		Pdot.col(2) = binomialCoeff(n, 2) * (-pow((t-1),2)*t*(5*t-2));
		Pdot.col(3) = binomialCoeff(n, 3) * ((t-1) * pow(t, 2) * (5*t-3));
		Pdot.col(4) = binomialCoeff(n, 4) * (-pow(t, 3)*(5*t-4));
		Pdot.col(5) = binomialCoeff(n, 5) * (5 * pow(t, 4));

		Pddot.col(0) = binomialCoeff(n, 0) * (-20*pow(t-1,3));
		Pddot.col(1) = binomialCoeff(n, 1) * (3*pow(t-1,2)*(5*t-1)+5*pow(t-1,3));
		Pddot.col(2) = binomialCoeff(n, 2) * (-2*(t-1)*(10*pow(t, 2)-8*t+1));
		Pddot.col(3) = binomialCoeff(n, 3) * (2 * t * (10 * pow(t, 2) - 12*t + 3));
		Pddot.col(4) = binomialCoeff(n, 4) * (-4 * pow(t, 2) * (5*t-3));
		Pddot.col(5) = binomialCoeff(n, 5) * (20 * pow(t, 3));


		Pdddot.col(0) = binomialCoeff(n, 0) * (-60*pow(t-1,2));
		Pdddot.col(1) = binomialCoeff(n, 1) * (12*(t-1)*(5*t-3));
		Pdddot.col(2) = binomialCoeff(n, 2) * (-60*pow(t, 2)+72*t-18);
		Pdddot.col(3) = binomialCoeff(n, 3) * (60 * pow(t, 2)-48*t+6);
		Pdddot.col(4) = binomialCoeff(n, 4) * ( -12*t*(5*t-2));
		Pdddot.col(5) = binomialCoeff(n, 5) * (60 * pow(t, 2));

		Pddddot.col(0) = binomialCoeff(n, 0) * (-120*(t-1));
		Pddddot.col(1) = binomialCoeff(n, 1) * (120*t - 96);
		Pddddot.col(2) = binomialCoeff(n, 2) * (72 - 120 * t);
		Pddddot.col(3) = binomialCoeff(n, 3) * (120 * t - 48);
		Pddddot.col(4) = binomialCoeff(n, 4) * (24 - 120*t);			
		Pddddot.col(5) = binomialCoeff(n, 5) * (120 * pow(t, 1));
	}
	else{
		ROS_ERROR_STREAM("Invalid degree. Only 5 and 10 available");
	}

	s.a = P;
	s.b = Pdot / l;
	s.c = Pddot / (l * l);
	s.d = Pdddot / (l * l * l);
	s.e = Pddddot / (l * l * l * l);

	return s;
}

Eigen :: ArrayXXd block_diag(Eigen :: ArrayXXd arr1, Eigen :: ArrayXXd arr2)
{
	Eigen :: ArrayXXd temp(arr1.rows() + arr2.rows(), arr1.cols() + arr2.cols());

	temp = 0.0;
	temp.topRows(arr1.rows()).leftCols(arr1.cols()) = arr1;
	temp.bottomRows(arr2.rows()).rightCols(arr2.cols()) = arr2;

	return temp;
}

five_var computeBernstein(Eigen :: ArrayXXd tot_time, double t_plan, int num, int pieces, int degree)
{
	five_var PPP;
	for(int i = 0; i < pieces; i++){

		tot_time = Eigen :: ArrayXXd(num/pieces, 1);
		tot_time.col(0).setLinSpaced(num/pieces, t_plan/pieces*i, t_plan/pieces*(i+1));

		five_var WWW;
		// WWW = bernsteinCoeffOrder10(degree, tot_time(i*num/pieces), tot_time((i+1)*num/pieces-1), (tot_time.topRows((i+1)*num/pieces)).bottomRows(num/pieces), num/pieces);
		WWW = bernsteinCoeff(degree, tot_time(0), tot_time(tot_time.rows()-1), tot_time, num/pieces);

		if(i==0){
			PPP.a = WWW.a;
			PPP.b = WWW.b;
			PPP.c = WWW.c;
			PPP.d = WWW.d;
			PPP.e = WWW.e;
		}
		else{
			PPP.a =  block_diag(PPP.a, WWW.a);
			PPP.b =  block_diag(PPP.b, WWW.b);
			PPP.c =  block_diag(PPP.c, WWW.c);
			PPP.d =  block_diag(PPP.d, WWW.d);
			PPP.e =  block_diag(PPP.e, WWW.e);
		}
	}
	
	return PPP;
}