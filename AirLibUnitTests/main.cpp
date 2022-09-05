
#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"

int main()
{
    using namespace msr::airlib;

    std::unique_ptr<TestBase> tests[] = {
        std::unique_ptr<TestBase>(new QuaternionTest()),
        std::unique_ptr<TestBase>(new CelestialTest()),
        std::unique_ptr<TestBase>(new SettingsTest()),
        std::unique_ptr<TestBase>(new SimpleFlightTest())
        //,
        //std::unique_ptr<TestBase>(new PixhawkTest()),
        //std::unique_ptr<TestBase>(new WorkerThreadTest())
    };

    std::chrono::time_point<std::chrono::system_clock> start, end;

    start = std::chrono::system_clock::now();
    for (auto& test : tests)
        test->run();
    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time (s): " << elapsed_seconds.count() << "\n";
    std::cout << "elapsed time (min): " << elapsed_seconds.count() / 60.0f << "\n";

    return 0;
}