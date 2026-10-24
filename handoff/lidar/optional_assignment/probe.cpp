#include <Lidar/Tracking/OptionalAssignment.hpp>
#include <iostream>
#include <cassert>
#include <limits>
int main() {
    using optional_assignment::Solve;
    assert(Solve({},3).empty());
    assert(Solve({{},{}},3)==std::vector<int>({-1,-1}));
    assert(Solve({{3}},3)==std::vector<int>({-1}));
    assert(Solve({{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),-1}},3)==std::vector<int>({-1}));
    for(double l: {0.,-1.,std::numeric_limits<double>::infinity()}) {
        bool threw=false;try{Solve({{1}},l);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    }
    bool threw=false;try{Solve({{1},{1,2}},3);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    int mode,n,m;
    while(std::cin>>mode>>n>>m) {
        std::vector<std::vector<double>> c(n,std::vector<double>(m));
        for(auto& row:c)for(auto& x:row)if(!(std::cin>>x))return 2;
        std::vector<int> a(n,-1);
        if(mode)a=Solve(c,3);
        else if(n && m) {
            AssignmentProblemSolver s;s.Solve(c,a,AssignmentProblemSolver::optimal);
            for(int i=0;i<n;++i)if(a[i]>=0 && c[i][a[i]]>=3)a[i]=-1;
        }
        std::cout<<n;for(int j:a)std::cout<<' '<<j;std::cout<<'\n';
    }
}
