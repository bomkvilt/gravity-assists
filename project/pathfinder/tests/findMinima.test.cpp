#include "gtest/gtest.h"
#include "gsl/gsl_multimin.h"



struct findMinima_tests : public testing::Test
{};

namespace findMinima_tests_funcs
{
	double my_f(const gsl_vector* v, void* params)
	{
		double x, y;
		double* p = (double*)params;
		x = gsl_vector_get(v, 0);
		y = gsl_vector_get(v, 1);
		return p[2] * (x - p[0]) * (x - p[0]) + p[3] * (y - p[1]) * (y - p[1]) + p[4];
	};

	void my_df(const gsl_vector* v, void* params, gsl_vector* df)
	{
		double x, y;
		double* p = (double*)params;
		x = gsl_vector_get(v, 0);
		y = gsl_vector_get(v, 1);
		gsl_vector_set(df, 0, 2.0 * p[2] * (x - p[0]));
		gsl_vector_set(df, 1, 2.0 * p[3] * (y - p[1]));
	};

	void my_fdf(const gsl_vector* x, void* params, double* f, gsl_vector* df)
	{
		*f = my_f(x, params);
		my_df(x, params, df);
	};
}

TEST_F(findMinima_tests, with_direvative)
{
	using namespace findMinima_tests_funcs;

	// x0, y0, /a, 1/b, z0
	double par[5] = { 1.0, 2.0, 10.0, 20.0, 30.0 };

	gsl_multimin_function_fdf my_func;
	my_func.n = 2;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = par;

	auto x = gsl_vector_alloc(2);
	gsl_vector_set(x, 0, 5.0);
	gsl_vector_set(x, 1, 7.0);

	auto T = gsl_multimin_fdfminimizer_conjugate_fr;
	auto s = gsl_multimin_fdfminimizer_alloc(T, 2);

	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 1e-4);

	size_t iter = 1;
	int  status = 0;
	do
	{
		status = gsl_multimin_fdfminimizer_iterate(s);
		if (status)
		{
			break;
		}

		status = gsl_multimin_test_gradient(s->gradient, 1e-3);

	} while (status == GSL_CONTINUE && iter++ < 100);

	ASSERT_FLOAT_EQ(s->f, par[4]);
	ASSERT_EQ(iter-1, 12);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);
}


TEST_F(findMinima_tests, without_direvative)
{
	using namespace findMinima_tests_funcs;

	// x0, y0, /a, 1/b, z0
	double par[5] = { 1.0, 2.0, 10.0, 20.0, 30.0 };

	/* Starting point */
	auto x = gsl_vector_alloc(2);
	gsl_vector_set(x, 0, 5.0);
	gsl_vector_set(x, 1, 7.0);

	/* Set initial step sizes to 1 */
	auto ss = gsl_vector_alloc(2);
	gsl_vector_set_all(ss, 1.0);

	/* Initialize method and iterate */
	gsl_multimin_function minex_func;
	minex_func.n = 2;
	minex_func.f = my_f;
	minex_func.params = par;

	auto T = gsl_multimin_fminimizer_nmsimplex2;
	auto s = gsl_multimin_fminimizer_alloc(T, 2);
	gsl_multimin_fminimizer_set(s, &minex_func, x, ss);

	size_t iter = 1;
	double size = 0;
	int  status = 0;
	do
	{
		status = gsl_multimin_fminimizer_iterate(s);
		if (status)
		{
			break;
		}

		size = gsl_multimin_fminimizer_size(s);
		status = gsl_multimin_test_size(size, 1e-2);
	} while (status == GSL_CONTINUE && iter++ < 100);

	ASSERT_FLOAT_EQ(int(s->fval * 1e2), int(par[4] * 1e2));
	ASSERT_EQ(iter, 24);

	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free(s);
}
