#include "gtest/gtest.h"
#include "defer.hpp"


TEST(tests, defer_explicit)
{
	int i = 1;
	{
		Defer _([&i]()
		{
			i = 2;
		});
	}
	ASSERT_EQ(i, 2);
}

TEST(tests, defer_macro)
{
	int i = 1;
	{
		DEFER(_)[&i]()
		{
			i = 2;
		};
	}
	ASSERT_EQ(i, 2);
}