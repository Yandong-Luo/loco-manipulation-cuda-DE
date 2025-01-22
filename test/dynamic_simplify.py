# https://courses.ece.ucsb.edu/ECE594/594D_W10Byl/hw/cartpole_eom.pdf
# the verification of simplify.
from sympy import symbols, sin, cos, simplify, Eq, solve

# # 定义符号变量
# Fx, M, mp, L, g, theta, dot_theta, ddot_x, ddot_theta = symbols(
#     'Fx M mp L g theta dot_theta ddot_x ddot_theta'
# )

# # 方程 1：水平方向力平衡
# eq1 = Eq(Fx, (M + mp) * ddot_x + mp * L * sin(theta) * dot_theta**2 - mp * L * cos(theta) * ddot_theta)

# # 方程 2：角运动方程
# eq2 = Eq(0, mp * L**2 * ddot_theta - mp * L * cos(theta) * ddot_x - mp * g * L * sin(theta))

# # 从 eq2 解出 ddot_theta
# ddot_theta_expr = solve(eq2, ddot_theta)[0]

# # 将 ddot_theta 代入 eq1，求解 ddot_x
# eq1_sub = eq1.subs(ddot_theta, ddot_theta_expr)
# ddot_x_expr = solve(eq1_sub, ddot_x)[0]

# # 验证 ddot_theta，替换 ddot_x
# ddot_theta_verified = simplify(ddot_theta_expr.subs(ddot_x, ddot_x_expr))

# # 输出结果
# print("ddot_x = ", ddot_x_expr)
# print("ddot_theta = ", ddot_theta_verified)


# https://richardrl.github.io/lyapunov-cart/lyapunov_controller.pdf
mc, mp, l, theta, ddot_x, ddot_theta, dot_theta, u, lam1, lam2, g = symbols(
    "mc mp l theta ddot_x ddot_theta dot_theta u lam1 lam2 g"
)

# Equation (8)
eq8 = Eq((mc + mp) * ddot_x - mp * l * cos(theta) * ddot_theta + mp * l * dot_theta**2 * sin(theta) - u + lam1 - lam2, 0)

# Equation (9)
eq9 = Eq(l * (-mp * cos(theta) * ddot_x + mp * l * ddot_theta - mp * g * sin(theta) - cos(theta) * lam1 + cos(theta) * lam2), 0)

# 从 eq9 解出 ddot_theta
ddot_theta_expr = solve(eq9, ddot_theta)[0]

# 将 ddot_theta 代入 eq8，求解 ddot_x
eq8_sub = eq8.subs(ddot_theta, ddot_theta_expr)
ddot_x_expr = solve(eq8_sub, ddot_x)[0]

ddot_theta_verified = simplify(ddot_theta_expr.subs(ddot_x, ddot_x_expr))

# 输出结果
print("ddot_x = ", ddot_x_expr)
print("ddot_theta = ", ddot_theta_verified)