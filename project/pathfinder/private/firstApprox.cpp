#include "firstApprox.hpp"
#include "firstApproxUtiles.hpp"



FirstApprox::FirstApprox(Planet& A, Planet& B, Planet& C, float t0, int n)
	: A(A), B(B), C(C)
	, t0(t0)
	, te(B.GetPeriod() / 2 + t0)
	, ts(B.GetPeriod() / 2 / n)
{
	A.SetTime(t0);
	B.SetTime(t0);
	C.SetTime(t0);
}


auto FirstApprox::FindTrajectories(const std::vector<float>& df0s) -> std::vector<FirstApproxUtiles::TragectoryFinder>
{
	// departure planet's params
	A.SetTime(t0);
	auto R0 = A.GetLocation();
	auto r0 = R0.Size();
	auto Q0 = 0.f;

	// barycenter's params
	auto M = C.GetGravParam();

	// fallaback's time finder
	auto n = 30;				// 
	auto TB = B.GetPeriod();	// 
	auto te = TB / 2 + t0;		// 
	auto ts = TB / 2 / n;		// 		
	auto finder = FirstApproxUtiles::TragectoryFinder(R0, Q0, t0, te, ts, M);

	auto solutions = std::vector<FirstApproxUtiles::TragectoryFinder>();

	// possible velocity's direction variations
	// \note: we try to find a meating point for each of the directions
	// \todo: generate the points automaticaly
	for (auto df0 : df0s)
	{
		auto solver = finder;
		if (solver.FindTragectory(B, Q0 + df0))
		{
			solutions.push_back(solver);
		}
	}
	return solutions;
}
