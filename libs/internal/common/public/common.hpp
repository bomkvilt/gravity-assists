#ifndef COMMON__COMMON_HPP
#define COMMON__COMMON_HPP

#include <type_traits>


#define CONST_FUNCTION_ENTERY(function_with_args) \
	using __this_c_no_p__   = std::remove_pointer_t<decltype(this)>;	\
	using __this_m_no_p__   = std::remove_const_t<__this_c_no_p__>;		\
	using __this_m_with_P__ = std::add_pointer_t<__this_c_no_p__>;		\
	return const_cast<__this_m_with_P__>(this)->function_with_args		\
/**/

#endif //!COMMON__COMMON_HPP
