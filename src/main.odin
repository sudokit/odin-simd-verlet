package main

import "core:fmt"
import "core:math"
import ln "core:math/linalg"
import "core:math/rand"
import "core:reflect"
import "core:simd"
import "core:time"

import rl "vendor:raylib"

import tracy "shared:odin-tracy"
import simdu "shared:simd_utils"

TRACY_ENABLE :: #config(TRACY_ENABLE, ODIN_DEBUG)

ONEF32 :: simd.f32x8(1)
ONEU32 :: simd.u32x8(1)
ZEROF32 :: simd.f32x8(0)
ZEROU32 :: simd.u32x8(0)

N :: 2e2 // this is going to be multiplied by 8
SUBSTEPS :: 8
DT :: 3e-4
G :: simd.f32x8(1000)

CVec2 :: [2]simd.f32x8

ParticleCluster :: struct {
	pos:     CVec2,
	pos_old: CVec2,
	accel:   CVec2,
	radius:  simd.f32x8,
}

random_simd :: proc(min, max: f32) -> simd.f32x8 {
	tmp: [8]f32
	for i in 0 ..< 8 {
		tmp[i] = rand.float32_range(min, max)
	}
	return simd.from_array(tmp)
}

random_cluster :: proc(window_dims, lim_vel: [2]f32, dt: f32) -> ParticleCluster {
	pos := CVec2{random_simd(0, window_dims.x), random_simd(0, window_dims.y)}
	vel := CVec2{random_simd(lim_vel.x, lim_vel.y), random_simd(lim_vel.x, lim_vel.y)}
	// vel := CVec2(0)
	dtl := simd.f32x8(dt)
	return {pos = pos, pos_old = pos - (vel * dtl), accel = 0, radius = random_simd(3, 3)}
}

Particles :: struct {
	clusters: []ParticleCluster,
}

W_DIMS: [2]f32 = {1800, 1000}

main :: proc() {
	tracy.SetThreadName("main")

	ps := Particles {
		clusters = make([]ParticleCluster, N),
	};defer delete(ps.clusters)

	R :: 8
	y := f32(R * 3)
	x := f32(R * 3)
	for i in 0 ..< N {
		pos_x: [8]f32
		pos_y: [8]f32
		for k in 0 ..< 8 {
			pos_x[k] = x
			pos_y[k] = y
			x += R * 3
			if x >= W_DIMS.x {
				x = R * 3
				y += R * 3
			}
		}
		pos :=
			CVec2{simd.from_array(pos_x), simd.from_array(pos_y)} +
			CVec2{random_simd(-1, 1), random_simd(-1, 1)}
		ps.clusters[i].pos = pos
		ps.clusters[i].pos_old = pos
		ps.clusters[i].accel = 0
		ps.clusters[i].radius = simd.f32x8(R)
	}

	rl.InitWindow(i32(W_DIMS.x), i32(W_DIMS.y), "cool");defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	buffer := rl.LoadRenderTexture(
		i32(W_DIMS.x),
		i32(W_DIMS.y),
	);defer rl.UnloadRenderTexture(buffer)

	for !rl.WindowShouldClose() && !rl.IsKeyPressed(.ESCAPE) {
		// update
		{
			tracy.ZoneN("Physics update")
			subdt := rl.GetFrameTime() / SUBSTEPS
			for substep in 0 ..< SUBSTEPS {
				tracy.ZoneN("Physics substep")
				for &cluster in ps.clusters {
					// gravity
					cluster.accel.y += G
				}

				{
					tracy.ZoneN("Collision check")
					resp_coef: f32 = 1 - 0.3
					resp_coefv := simd.f32x8(resp_coef)

					for &cluster in ps.clusters {
						// so what we do is go trough each particle and broadcast its pos into a f32x8 and then check that against 8 other ball positions

						for i in 0 ..< len(cluster.pos.x) {
							radius := simd.f32x8(simd.extract(cluster.radius, i))
							// NOTE: this doesnt work :/
							for &ocluster in ps.clusters {
								if ocluster == cluster do continue
								pos := CVec2 {
									simd.extract(cluster.pos.x, i),
									simd.extract(cluster.pos.y, i),
								}
								v := pos - ocluster.pos
								dist2 := v.x * v.x + v.y * v.y
								min_dist := radius + ocluster.radius
								mask := simd.lanes_lt(dist2, min_dist * min_dist) & ONEU32
								if (simd.reduce_or(mask) == 1) {
									// okay we got a collision

									dist := simd.sqrt(dist2)
									n :=
										v /
										simd.clamp(
											dist,
											simd.f32x8(math.F32_EPSILON),
											simd.f32x8(math.INF_F32),
										)
									mass_ratio_1 := radius / (radius + ocluster.radius)
									mass_ratio_2 := ocluster.radius / (radius + ocluster.radius)
									delta := simd.f32x8(0.5) * resp_coefv * (dist - min_dist)
									fmask := simd.f32x8(mask)

									calc1 := (n * (mass_ratio_2 * delta)) * fmask
									calc2 := (n * (mass_ratio_1 * delta)) * fmask
									pos -= calc1
									ocluster.pos += calc2

									{

										extracted := CVec2 {
											simd.extract(cluster.pos.x, i),
											simd.extract(cluster.pos.y, i),
										}
										dxdy := pos - extracted
										average_dx := simd.reduce_add_ordered(dxdy.x) / len(dxdy.x)
										average_dy := simd.reduce_add_ordered(dxdy.y) / len(dxdy.y)

										cluster.pos.x = simd.replace(
											cluster.pos.x,
											i,
											simd.extract(cluster.pos.x, i) + average_dx,
										)
										cluster.pos.y = simd.replace(
											cluster.pos.y,
											i,
											simd.extract(cluster.pos.y, i) + average_dy,
										)
									}
								}
							} // cluster 2

						} // each particle in cluster (collision with other particles)


						for i in 0 ..< len(cluster.pos.x) {
							radius := simd.f32x8(simd.extract(cluster.radius, i))


							// NOTE: this works :)
							// now we would need to do collision inside the simd vector
							// self collision. scalar loop
							mr := simd.extract(radius, 0)
							for j in 0 ..< len(cluster.pos.x) {
								mpos := [2]f32 {
									simd.extract(cluster.pos.x, i),
									simd.extract(cluster.pos.y, i),
								}
								other_pos := [2]f32 {
									simd.extract(cluster.pos.x, j),
									simd.extract(cluster.pos.y, j),
								}
								or := simd.extract(cluster.radius, j)
								if other_pos == mpos do continue
								v := mpos - other_pos
								dist2 := v.x * v.x + v.y * v.y
								min_dist := mr + or

								if dist2 < min_dist * min_dist {
									dist := math.sqrt(dist2)
									n := v / dist
									mratio1 := mr / (mr + or)
									mratio2 := or / (mr + or)
									delta := 0.5 * resp_coef * (dist - min_dist)

									calc1 := n * (mratio2 * delta)
									calc2 := n * (mratio1 * delta)
									cluster.pos.x = simd.replace(
										cluster.pos.x,
										i,
										mpos.x - calc1.x,
									)
									cluster.pos.y = simd.replace(
										cluster.pos.y,
										i,
										mpos.y - calc1.y,
									)

									cluster.pos.x = simd.replace(
										cluster.pos.x,
										j,
										other_pos.x + calc2.x,
									)
									cluster.pos.y = simd.replace(
										cluster.pos.y,
										j,
										other_pos.y + calc2.y,
									)
								}
							}
						} // each particle in current cluster (self collision)
					} // cluster 1
				} // collision

				for &cluster in ps.clusters {
					// constraint

					// edges
					right := simd.f32x8(W_DIMS.x)
					bottom := simd.f32x8(W_DIMS.y)
					radiuses := cluster.radius

					vel := cluster.pos - cluster.pos_old

					damping := simd.f32x8(1 - 0.3)

					// these give us which ones collided. 1 for which collided and 0 for which didnt. when we invert this we get the ones that didnt collide
					cr := simd.lanes_gt(cluster.pos.x + radiuses, right) & ONEU32
					cl := simd.lanes_lt(cluster.pos.x - radiuses, ZEROF32) & ONEU32

					if simd.reduce_or(cr) == 1 {
						// right edge (width)
						crf := simd.f32x8(cr)
						cluster.pos.x = simd.clamp(
							cluster.pos.x,
							simd.f32x8(-math.INF_F32),
							simd.f32x8(right) - radiuses,
						)
						cluster.pos_old.x +=
							((cluster.pos.x + vel.x * damping) - cluster.pos_old.x) * crf
					}
					if simd.reduce_or(cl) == 1 {
						// left edge (0)
						clf := simd.f32x8(cl)
						cluster.pos.x *= simd.f32x8(cl ~ ONEU32)
						cluster.pos.x += radiuses * clf
						cluster.pos_old.x +=
							((cluster.pos.x + vel.x * damping) - cluster.pos_old.x) * clf
					}

					cb := simd.lanes_gt(cluster.pos.y + radiuses, bottom) & ONEU32
					ct := simd.lanes_lt(cluster.pos.y - radiuses, ZEROF32) & ONEU32
					if simd.reduce_or(cb) == 1 {
						// bottom edge (height)
						cbf := simd.f32x8(cb)
						cluster.pos.y = simd.clamp(
							cluster.pos.y,
							simd.f32x8(-math.INF_F32),
							simd.f32x8(bottom) - radiuses,
						)
						cluster.pos_old.y +=
							((cluster.pos.y + vel.y * damping) - cluster.pos_old.y) * cbf
					}
					if simd.reduce_or(ct) == 1 {
						// top edge (0)
						ctf := simd.f32x8(ct)
						cluster.pos.y *= simd.f32x8(ct ~ ONEU32)
						cluster.pos.y += radiuses * ctf
						cluster.pos_old.y +=
							((cluster.pos.y + vel.y * damping) - cluster.pos_old.y) * ctf
					}
				} // consraint

				for &cluster in ps.clusters {
					// update
					// dt := simd.f32x8(DT)
					// dt := simd.f32x8(rl.GetFrameTime())
					dt := simd.f32x8(subdt)
					disp := cluster.pos - cluster.pos_old
					cluster.pos_old = cluster.pos
					cluster.pos = cluster.pos + disp + (cluster.accel / cluster.radius) * (dt * dt)
					cluster.accel = 0
				}
			} // substep
		} // zone end

		{
			tracy.ZoneN("Render")
			rl.BeginDrawing()
			rl.BeginTextureMode(buffer)
			rl.ClearBackground(rl.BLACK)
			// draw clusters
			for cluster in ps.clusters {
				comb := simdu.interleave(cluster.pos.x, cluster.pos.y)
				radiuses := simd.to_array(cluster.radius)
				index := 0
				for i in 0 ..< len(radiuses) {
					pos := [2]f32{comb[index], comb[index + 1]}
					rl.DrawCircleV(pos, radiuses[i], COLORS[i32(pos.y) % len(COLORS)])
					index += 2
				}
			}
			rl.EndTextureMode()
			rl.DrawTexturePro(
				buffer.texture,
				{0, W_DIMS.y, W_DIMS.x, -W_DIMS.y},
				{0, 0, W_DIMS.x, W_DIMS.y},
				{0, 0},
				0,
				rl.WHITE,
			)
			rl.DrawFPS(0, 0)
			rl.EndDrawing()
		}
	}
}


COLORS := [?]rl.Color {
	rl.LIGHTGRAY,
	rl.GRAY,
	rl.DARKGRAY,
	rl.YELLOW,
	rl.GOLD,
	rl.ORANGE,
	rl.PINK,
	rl.RED,
	rl.MAROON,
	rl.GREEN,
	rl.LIME,
	rl.DARKGREEN,
	rl.SKYBLUE,
	rl.BLUE,
	rl.DARKBLUE,
	rl.PURPLE,
	rl.VIOLET,
	rl.DARKPURPLE,
	rl.BEIGE,
	rl.WHITE,
	rl.MAGENTA,
	rl.RAYWHITE,
}

