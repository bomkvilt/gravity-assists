#ifndef COMMON__DEFER_HPP
#define COMMON__DEFER_HPP

#include <functional>


class Defer
{
public:
	Defer(std::function<void()> clb = nullptr)
		: clb(clb)
	{}

	~Defer()
	{
		clb();
	}

private:
	std::function<void()> clb;
};


#define DEFER(name) \
	Defer name =	\
/**/


#endif //!COMMON__DEFER_HPP
