#include "gtest/gtest.h"
#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_min.h>
#include "defer.hpp"


TEST(tests, min_scalar)
{
	auto fn1 = [](double x, void*) -> double
	{
		return cos(x) + 1.0;
	};
		
	auto F = gsl_function();
	F.function = fn1;
	F.params = nullptr;

	auto a = 0.0, b = 6.0;
	auto m = 2.0, m_expected = M_PI;
	auto T = gsl_min_fminimizer_brent;
	auto s = gsl_min_fminimizer_alloc(T);
	DEFER(_)[&s]() 
	{ 
		gsl_min_fminimizer_free(s); 
	};

	ASSERT_FALSE(gsl_min_fminimizer_set(s, &F, m, a, b));

	int iter = 0, max_iter = 100;
	for (; iter < max_iter; ++iter)
	{
		ASSERT_FALSE(gsl_min_fminimizer_iterate(s));
		a = gsl_min_fminimizer_x_lower(s);
		b = gsl_min_fminimizer_x_upper(s);
		m = gsl_min_fminimizer_x_minimum(s);

		if (b - a < 0.001)
		{
			break;
		}
	};
	ASSERT_FLOAT_EQ(m, m_expected);
}


TEST(tests, min_scalar_minOnBound)
{
	auto F = gsl_function();
	F.params = nullptr;
	F.function = [](double x, void*) -> double
	{
		return 10 - x;
	};

	auto a = 0.0, b = 6.0;
	auto m = 3.0, m_expected = 3.0;
	auto T = gsl_min_fminimizer_brent;
	auto s = gsl_min_fminimizer_alloc(T);
	DEFER(_)[&s]() 
	{ 
		gsl_min_fminimizer_free(s); 
	};

	// the function disables abort() function call
	gsl_set_error_handler_off();
	// if the m point's value grate or equal to bounds' ones
	// the finction preforms abort() or return errno != 0
	// the same s#?*t happends when the func return NAN ot b < a
	ASSERT_TRUE(gsl_min_fminimizer_set(s, &F, m, a, b));
}

TEST(tests, min_scalar_twoRoot)
{
	auto F = gsl_function();
	F.params = nullptr;
	F.function = [](double x, void*) -> double
	{
		return cos(x) + 0.0;
	};

	auto p3 = M_PI / 3;
	auto a = p3 * 0;
	auto b = p3 * 4.2;
	auto m = p3, m_expected = 2 * p3;
	auto T = gsl_min_fminimizer_brent;
	auto s = gsl_min_fminimizer_alloc(T);
	DEFER(_)[&s]()
	{
		gsl_min_fminimizer_free(s);
	};

	gsl_set_error_handler_off();
	// if the function's middle point is grater the bounds' ones
	// minimiser wont be initialized
	ASSERT_TRUE(gsl_min_fminimizer_set(s, &F, m, a, b));
}


TEST(tests, min_gradient)
{
	double par[5] = { 1.0, 2.0, 10.0, 20.0, 30.0 };
	
	gsl_multimin_function minex_func;
	minex_func.n = 2;
	minex_func.f = [](const gsl_vector* v, void* params)
	{
		double* p = (double*)params;
		auto x = gsl_vector_get(v, 0);
		auto y = gsl_vector_get(v, 1);
		auto X = (x - p[0]);
		auto Y = (y - p[1]);
		return p[2]*X*X + p[3]*Y*Y + p[4];
	};
	minex_func.params = par;

	auto x  = gsl_vector_alloc(2); // starting point
	auto ss = gsl_vector_alloc(2); // initial step sizes
	gsl_vector_set_all(ss, 1.0);
	gsl_vector_set(x, 0, 5.0);
	gsl_vector_set(x, 1, 7.0);

	auto T = gsl_multimin_fminimizer_nmsimplex2;
	auto s = gsl_multimin_fminimizer_alloc(T, 2);
	DEFER(_)[s, x, ss]()
	{
		gsl_vector_free(x );
		gsl_vector_free(ss);
		gsl_multimin_fminimizer_free(s);
	};

	ASSERT_FALSE(gsl_multimin_fminimizer_set(s, &minex_func, x, ss));

	int status = GSL_CONTINUE;
	for (int iter = 0, max_iter = 100; status == GSL_CONTINUE && iter < max_iter; ++iter)
	{
		if (status = gsl_multimin_fminimizer_iterate(s))
		{
			break;
		}
		auto size = gsl_multimin_fminimizer_size(s);
		status = gsl_multimin_test_size(size, 1e-2);
	}
	ASSERT_FALSE(status);
	EXPECT_NEAR(gsl_vector_get(s->x, 0), par[0], 1e-2);
	EXPECT_NEAR(gsl_vector_get(s->x, 1), par[1], 1e-2);
	EXPECT_NEAR(s->fval, par[4], 1e-2);
}


TEST(tests, min_gradient_scalar_twoRoots)
{
	gsl_multimin_function minex_func;
	minex_func.f = [](const gsl_vector* v, void* params)
	{
		auto t = gsl_vector_get(v, 0);
		return abs(cos(t) + 0.5);
	};
	minex_func.params = nullptr;
	minex_func.n = 1;

	auto p3 = M_PI / 3;
	auto m  = p3, m_exp = 2* p3, fm = 0.;
	auto x  = gsl_vector_alloc(1); // starting point
	auto ss = gsl_vector_alloc(1); // initial step sizes
	gsl_vector_set(ss, 0, 0.1);
	gsl_vector_set(x,  0, m);

	auto T = gsl_multimin_fminimizer_nmsimplex2;
	auto s = gsl_multimin_fminimizer_alloc(T, 1);
	DEFER(_)[s, x, ss]()
	{
		gsl_vector_free(x );
		gsl_vector_free(ss);
		gsl_multimin_fminimizer_free(s);
	};

	ASSERT_FALSE(gsl_multimin_fminimizer_set(s, &minex_func, x, ss));

	int status = GSL_CONTINUE;
	for (int iter = 0, max_iter = 100; status == GSL_CONTINUE && iter < max_iter; ++iter)
	{
		if (status = gsl_multimin_fminimizer_iterate(s))
		{
			break;
		}
		auto size = gsl_multimin_fminimizer_size(s);
		status = gsl_multimin_test_size(size, 1e-2);
	}

	// the function must find a clothet min
	ASSERT_FALSE(status);
	EXPECT_NEAR(s->fval, fm, 1e-2);
	EXPECT_NEAR(gsl_vector_get(s->x, 0), 1*m_exp, 1e-2);
}
