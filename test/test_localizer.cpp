#include <catch2/catch_test_macros.hpp>
//#include "ekf_localizer.hpp"

unsigned f(){ return 0; }

TEST_CASE( "Factorials are computed", "[factorial]" ) {
    REQUIRE( f() == 0 );
}
