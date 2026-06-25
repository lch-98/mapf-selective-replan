// ─────────────────────────────────────────────────────────────────
// tests/test_reservation_table.cpp
//
// 03_reservation_table.md에서 설명한 ReservationTable의 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include "mapf/reservation_table.hpp"

using namespace mapf;

TEST(ReservationTableTest, EmptyTableHasNoOwner) {
    ReservationTable table;
    EXPECT_FALSE(table.is_occupied(1, 1, 0));
    EXPECT_EQ(table.get_owner(1, 1, 0), std::nullopt);
}

TEST(ReservationTableTest, ReserveMarksOccupiedWithOwner) {
    ReservationTable table;
    table.reserve(1, 1, 0, /*agent_id=*/5);

    EXPECT_TRUE(table.is_occupied(1, 1, 0));
    EXPECT_EQ(table.get_owner(1, 1, 0), 5);
}

TEST(ReservationTableTest, ReserveWithoutAgentIdDefaultsToMinusOne) {
    ReservationTable table;
    table.reserve(1, 1, 0);  // agent_id 생략

    // "주인 모름(-1)"과 "점유 안 됨(nullopt)"은 다르다.
    EXPECT_TRUE(table.is_occupied(1, 1, 0));
    EXPECT_EQ(table.get_owner(1, 1, 0), -1);
}

TEST(ReservationTableTest, UnreserveClearsOccupancy) {
    ReservationTable table;
    table.reserve(1, 1, 0, 5);
    table.unreserve(1, 1, 0);

    EXPECT_FALSE(table.is_occupied(1, 1, 0));
    EXPECT_EQ(table.get_owner(1, 1, 0), std::nullopt);
}

TEST(ReservationTableTest, UnreserveIfOwnedByOnlyRemovesMatchingOwner) {
    ReservationTable table;
    table.reserve(1, 1, 0, 5);

    // 주인이 다르면 지워지지 않는다.
    table.unreserve_if_owned_by(1, 1, 0, 9);
    EXPECT_TRUE(table.is_occupied(1, 1, 0));
    EXPECT_EQ(table.get_owner(1, 1, 0), 5);

    // 주인이 맞으면 지워진다.
    table.unreserve_if_owned_by(1, 1, 0, 5);
    EXPECT_FALSE(table.is_occupied(1, 1, 0));
}

TEST(ReservationTableTest, ReserveIfUnownedRefusesToOverwriteOtherOwner) {
    ReservationTable table;
    table.reserve(1, 1, 0, 5);

    // 이미 다른 agent_id(5)가 점유 중이면 덮어쓰지 않고 false를 반환한다.
    bool ok = table.reserve_if_unowned(1, 1, 0, 9);
    EXPECT_FALSE(ok);
    EXPECT_EQ(table.get_owner(1, 1, 0), 5);
}

TEST(ReservationTableTest, ReserveIfUnownedSucceedsWhenEmptyOrSameOwner) {
    ReservationTable table;

    // 비어있으면 성공한다.
    EXPECT_TRUE(table.reserve_if_unowned(1, 1, 0, 5));
    EXPECT_EQ(table.get_owner(1, 1, 0), 5);

    // 이미 자기 자신의 것이면 다시 호출해도 성공한다(같은 칸을 여러 시각
    // 반복 예약하는 Tail Reservation 루프에서 필요한 동작).
    EXPECT_TRUE(table.reserve_if_unowned(1, 1, 0, 5));
    EXPECT_EQ(table.get_owner(1, 1, 0), 5);
}

TEST(ReservationTableTest, ReservationIsIsolatedByCoordinateAndTime) {
    ReservationTable table;
    table.reserve(1, 1, 0, 5);

    // 다른 시각, 다른 칸은 영향받지 않는다.
    EXPECT_FALSE(table.is_occupied(1, 1, 1));
    EXPECT_FALSE(table.is_occupied(2, 1, 0));
}

TEST(ReservationTableTest, ReserveEdgeBlocksBothDirections) {
    ReservationTable table;
    table.reserve_edge(1, 1, 2, 1, /*t=*/0);

    EXPECT_TRUE(table.is_edge_occupied(1, 1, 2, 1, 0));  // 정방향
    EXPECT_TRUE(table.is_edge_occupied(2, 1, 1, 1, 0));  // 역방향
}

TEST(ReservationTableTest, EdgeReservationIsIsolatedByTime) {
    ReservationTable table;
    table.reserve_edge(1, 1, 2, 1, 0);

    EXPECT_FALSE(table.is_edge_occupied(1, 1, 2, 1, 1));  // 다른 시각
}

TEST(ReservationTableTest, ClearRemovesAllReservations) {
    ReservationTable table;
    table.reserve(1, 1, 0, 5);
    table.reserve_edge(1, 1, 2, 1, 0);

    table.clear();

    EXPECT_FALSE(table.is_occupied(1, 1, 0));
    EXPECT_FALSE(table.is_edge_occupied(1, 1, 2, 1, 0));
}
