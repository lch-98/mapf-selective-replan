// analyze_run 실행 파일 회귀 테스트(31.22): 같은 시각에 같은 로봇 줄이 두 번 있어도 "자기 자신과 겹침"으로 세지 않는다.
// 진짜로 겹친 두 로봇은 여전히 잡는다(양성 대조).
#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

namespace {

std::string run(const std::string& csv_body) {
    const std::string dir = ::testing::TempDir();
    std::ofstream(dir + "/map.txt") << "....\n....\n";
    std::ofstream(dir + "/run.csv") << "t,robot,x,y,yaw,kind,held,seg_sx,seg_sy,seg_gx,seg_gy\n" << csv_body;
    const std::string cmd = std::string(ANALYZE_RUN_PATH) + " --map " + dir + "/map.txt --cell 0.5 --log " + dir + "/run.csv";
    std::array<char, 512> buf{};
    std::string out;
    std::unique_ptr<FILE, decltype(&pclose)> p(popen(cmd.c_str(), "r"), pclose);
    while (p && fgets(buf.data(), buf.size(), p.get())) out += buf.data();
    return out;
}

}  // namespace

TEST(AnalyzeRun, DuplicateRowsOfSameRobotAreNotSelfOverlap) {
    // 2행 4열 격자. robot_1은 1행 0열 중심(0.25, 0.25), robot_2는 0행 3열 중심(1.75, 0.75). 같은 시각 줄이 두 번씩.
    const std::string rows =
        "1.000,robot_1,0.25,0.25,0,IDLE,0:1,0.25,0.25,0.25,0.25\n"
        "1.000,robot_2,1.75,0.75,0,IDLE,3:0,1.75,0.75,1.75,0.75\n"
        "1.000,robot_1,0.25,0.25,0,IDLE,0:1,0.25,0.25,0.25,0.25\n"
        "1.000,robot_2,1.75,0.75,0,IDLE,3:0,1.75,0.75,1.75,0.75\n";
    const std::string out = run(rows);
    EXPECT_EQ(out.find("robot_1|robot_1"), std::string::npos) << out;
    EXPECT_NE(out.find("\"robot_1|robot_2\": {\"checked\": 1, \"overlap_samples\": 0"), std::string::npos) << out;
}

TEST(AnalyzeRun, RealOverlapIsStillCaught) {
    // 두 로봇을 0.1 m 떨어뜨려 놓음 → 몸이 겹침
    const std::string rows =
        "1.000,robot_1,0.25,0.25,0,IDLE,0:1,0.25,0.25,0.25,0.25\n"
        "1.000,robot_2,0.35,0.25,0,IDLE,0:1,0.35,0.25,0.35,0.25\n";
    const std::string out = run(rows);
    EXPECT_NE(out.find("\"robot_1|robot_2\": {\"checked\": 1, \"overlap_samples\": 1"), std::string::npos) << out;
}
