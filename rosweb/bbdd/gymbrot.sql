-- phpMyAdmin SQL Dump
-- version 4.9.5deb2
-- https://www.phpmyadmin.net/
--
-- Servidor: localhost:3306
-- Tiempo de generación: 12-05-2025 a las 18:18:17
-- Versión del servidor: 10.3.39-MariaDB-0ubuntu0.20.04.2
-- Versión de PHP: 7.4.3-4ubuntu2.29

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
SET AUTOCOMMIT = 0;
START TRANSACTION;
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

-- Base de datos: `gymbrot`
--
CREATE DATABASE IF NOT EXISTS gymbrot;
USE gymbrot;
--

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `activity_types`
--

CREATE TABLE `activity_types` (
  `id` int(11) NOT NULL,
  `name` varchar(50) NOT NULL,
  `description` varchar(255) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `activity_types`
--

INSERT INTO `activity_types` (`id`, `name`, `description`) VALUES
(1, 'login', 'Inicio de sesión del usuario'),
(2, 'logout', 'Cierre de sesión del usuario'),
(3, 'exercise', 'Realización de ejercicio'),
(4, 'profile_update', 'Actualización de perfil'),
(5, 'robot_interaction', 'Interacción con el robot');

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `exercises`
--

CREATE TABLE `exercises` (
  `id` int(11) NOT NULL,
  `name` varchar(100) NOT NULL,
  `description` text NOT NULL,
  `machine` tinyint(3) UNSIGNED NOT NULL,
  `muscle` tinyint(3) UNSIGNED NOT NULL,
  `video` varchar(255) DEFAULT NULL,
  `image` varchar(255) DEFAULT NULL,
  `task` varchar(500) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `exercises`
--

INSERT INTO `exercises` (`id`, `name`, `description`, `machine`, `muscle`, `video`, `image`, `task`) VALUES
(1, 'Press de banca', 'El press de banca es un ejercicio fundamental para desarrollar la fuerza y masa muscular del pecho. Se realiza acostado en un banco horizontal, sujetando una barra con las manos separadas al ancho de los hombros. Descender la barra controladamente hasta tocar el pecho y luego empujar hacia arriba manteniendo la estabilidad corporal. Trabaja principalmente pectorales, deltoides anteriores y tríceps.', 1, 1, 'https://www.youtube.com/watch?v=vcBig73ojpE', 'https://www.fisioterapiaconmueve.com/wp-content/uploads/2018/04/1.jpg', 'Realizar 4 series progresivas: \r\n1. 12 repeticiones con 50% RM \r\n2. 10 repeticiones con 70% RM \r\n3. '),
(2, 'Yoga Vinyasa', 'Práctica dinámica de yoga que sincroniza movimiento y respiración. Incluye secuencias fluidas como el saludo al sol, posturas de equilibrio y torsiones. Mejora flexibilidad, equilibrio y concentración. Ideal para movilidad articular y recuperación activa.', 2, 4, 'https://www.youtube.com/watch?v=4vTJGUQ5QDA', 'https://hips.hearstapps.com/hmg-prod/images/yoga-vinyasa-portada-womenshealth-1622106042.jpg', 'Sesión de 45 minutos:\r\n- 10 min calentamiento con saludo al sol\r\n- 25 min secuencia de posturas (gue'),
(3, 'Sentadillas con barra', 'Ejercicio rey para desarrollo de piernas y glúteos. Colocar la barra sobre trapecios, mantener espalda neutral y descender flexionando rodillas hasta paralelo. Enfocarse en técnica perfecta para prevenir lesiones. Trabaja cuádriceps, isquiotibiales y glúteos mayor.', 3, 5, 'https://www.youtube.com/watch?v=Dy28eq2PjcM', 'https://vitarclub.es/wp-content/uploads/2023/01/close-up-on-woman-doing-crossfit-workout-scaled.jpg', 'Pirámide de intensidad:\r\n1. 15 repeticiones con barra vacía\r\n2. 12 repeticiones +20kg\r\n3. 10 repetic'),
(4, 'Press de banca', 'El press de banca es un ejercicio fundamental para desarrollar la fuerza y masa muscular del pecho. Se realiza acostado en un banco horizontal, sujetando una barra con las manos separadas al ancho de los hombros. Descender la barra controladamente hasta tocar el pecho y luego empujar hacia arriba manteniendo la estabilidad corporal. Trabaja principalmente pectorales, deltoides anteriores y tríceps.', 1, 1, 'https://www.youtube.com/watch?v=vcBig73ojpE', 'https://blogscdn.thehut.net/app/uploads/sites/450/2020/08/militari-press_1598867922.jpg', 'Series: 4 progresivas | Reps: 12-10-8-6 | Carga: 50%-90% RM | Descanso: 90s'),
(5, 'Yoga Vinyasa', 'Práctica dinámica de yoga que sincroniza movimiento y respiración. Incluye secuencias fluidas como el saludo al sol, posturas de equilibrio y torsiones. Mejora flexibilidad, equilibrio y concentración. Ideal para movilidad articular y recuperación activa.', 2, 4, 'https://www.youtube.com/watch?v=4vTJGUQ5QDA', 'https://ignisfisioterapiagirona.cat/wp-content/uploads/2020/01/pilates-girona.jpg', 'Duración: 45min | Frecuencia: 3x/semana | Componentes: Calentamiento, posturas, relajación'),
(6, 'Sentadillas con barra', 'Ejercicio rey para desarrollo de piernas y glúteos. Colocar la barra sobre trapecios, mantener espalda neutral y descender flexionando rodillas hasta paralelo. Enfocarse en técnica perfecta para prevenir lesiones. Trabaja cuádriceps, isquiotibiales y glúteos mayor.', 3, 5, 'https://www.youtube.com/watch?v=Dy28eq2PjcM', 'https://vitruve.fit/wp-content/uploads/2024/05/Main-differences-Between-Squats-Deadlifts-1.jpg', 'Pirámide: 15-12-10-8 reps | Carga: +0kg/+20kg/+40kg/+60kg | Descanso: 2min');

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `machines`
--

CREATE TABLE `machines` (
  `id` tinyint(3) UNSIGNED NOT NULL,
  `locX` float NOT NULL,
  `locY` float NOT NULL,
  `orientation` float NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `machines`
--

INSERT INTO `machines` (`id`, `locX`, `locY`, `orientation`) VALUES
(1, 12.5, 8.3, 90),
(2, 15, 3.7, 180),
(3, 7.2, 10.1, 45);

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `muscles`
--

CREATE TABLE `muscles` (
  `id` tinyint(3) UNSIGNED NOT NULL,
  `name` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `muscles`
--

INSERT INTO `muscles` (`id`, `name`) VALUES
(1, 'Pectorales'),
(2, 'Bíceps'),
(3, 'Tríceps'),
(4, 'Abdominales'),
(5, 'Cuádriceps');

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `routine`
--

CREATE TABLE `routine` (
  `id` int(11) NOT NULL,
  `name` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `routine`
--

INSERT INTO `routine` (`id`, `name`) VALUES
(1, 'Rutina tren superior'),
(2, 'Rutina tren inferior'),
(3, 'Rutina full body');

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `routine_exercise`
--

CREATE TABLE `routine_exercise` (
  `id` int(11) NOT NULL,
  `routine` int(11) NOT NULL,
  `exercise` int(11) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `routine_exercise`
--

INSERT INTO `routine_exercise` (`id`, `routine`, `exercise`) VALUES
(4, 1, 1),
(5, 1, 3),
(6, 2, 2);

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `users`
--

CREATE TABLE `users` (
  `id` int(11) NOT NULL,
  `email` varchar(100) NOT NULL,
  `password` varchar(255) NOT NULL,
  `user_type` int(11) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `users`
--

INSERT INTO `users` (`id`, `email`, `password`, `user_type`) VALUES
(1, 'admin@gymbrot.com', 'admin123', 1),
(2, 'usuario1@gymbrot.com', 'user123', 2);

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `user_activities`
--

CREATE TABLE `user_activities` (
  `id` int(11) NOT NULL,
  `user_id` int(11) NOT NULL,
  `activity_type` varchar(50) NOT NULL,
  `description` text NOT NULL,
  `created_at` datetime NOT NULL DEFAULT current_timestamp(),
  `robot_id` tinyint(3) UNSIGNED DEFAULT NULL,
  `exercise_id` int(11) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `user_activities`
--

INSERT INTO `user_activities` (`id`, `user_id`, `activity_type`, `description`, `created_at`, `robot_id`, `exercise_id`) VALUES
(1, 2, 'login', 'Usuario inició sesión', '2025-05-12 16:56:32', NULL, NULL),
(2, 2, 'exercise', 'Completó Press de banca', '2025-05-12 16:56:32', 1, 1),
(3, 2, 'logout', 'Usuario cerró sesión', '2025-05-12 16:56:32', NULL, NULL),
(4, 2, 'profile_update', 'Usuario actualizó datos personales', '2025-05-12 16:56:32', NULL, NULL),
(5, 2, 'robot_interaction', 'Usuario configuró parámetros del robot', '2025-05-12 16:56:32', 1, NULL),
(6, 1, 'login', 'Administrador accedió al panel', '2025-05-12 16:56:32', NULL, NULL);

-- --------------------------------------------------------

--
-- Estructura de tabla para la tabla `user_type`
--

CREATE TABLE `user_type` (
  `id` int(11) NOT NULL,
  `rol` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Volcado de datos para la tabla `user_type`
--

INSERT INTO `user_type` (`id`, `rol`) VALUES
(1, 'Admin'),
(2, 'Usuario');

--
-- Índices para tablas volcadas
--

--
-- Indices de la tabla `activity_types`
--
ALTER TABLE `activity_types`
  ADD PRIMARY KEY (`id`),
  ADD UNIQUE KEY `name` (`name`);

--
-- Indices de la tabla `exercises`
--
ALTER TABLE `exercises`
  ADD PRIMARY KEY (`id`),
  ADD KEY `fk_exercises_muscles` (`muscle`),
  ADD KEY `fk_exercises_machines` (`machine`);

--
-- Indices de la tabla `machines`
--
ALTER TABLE `machines`
  ADD PRIMARY KEY (`id`);

--
-- Indices de la tabla `muscles`
--
ALTER TABLE `muscles`
  ADD PRIMARY KEY (`id`);

--
-- Indices de la tabla `routine`
--
ALTER TABLE `routine`
  ADD PRIMARY KEY (`id`);

--
-- Indices de la tabla `routine_exercise`
--
ALTER TABLE `routine_exercise`
  ADD PRIMARY KEY (`id`),
  ADD KEY `fk_routine_exercise_routine` (`routine`),
  ADD KEY `fk_routine_exercise_exercise` (`exercise`);

--
-- Indices de la tabla `users`
--
ALTER TABLE `users`
  ADD PRIMARY KEY (`id`),
  ADD UNIQUE KEY `email` (`email`),
  ADD KEY `fk_users_user_type` (`user_type`);

--
-- Indices de la tabla `user_activities`
--
ALTER TABLE `user_activities`
  ADD PRIMARY KEY (`id`),
  ADD KEY `fk_user_activities_user` (`user_id`),
  ADD KEY `fk_user_activities_exercise` (`exercise_id`);

--
-- Indices de la tabla `user_type`
--
ALTER TABLE `user_type`
  ADD PRIMARY KEY (`id`);

--
-- AUTO_INCREMENT de las tablas volcadas
--

--
-- AUTO_INCREMENT de la tabla `activity_types`
--
ALTER TABLE `activity_types`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=6;

--
-- AUTO_INCREMENT de la tabla `exercises`
--
ALTER TABLE `exercises`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=7;

--
-- AUTO_INCREMENT de la tabla `machines`
--
ALTER TABLE `machines`
  MODIFY `id` tinyint(3) UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=4;

--
-- AUTO_INCREMENT de la tabla `muscles`
--
ALTER TABLE `muscles`
  MODIFY `id` tinyint(3) UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=6;

--
-- AUTO_INCREMENT de la tabla `routine`
--
ALTER TABLE `routine`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=4;

--
-- AUTO_INCREMENT de la tabla `routine_exercise`
--
ALTER TABLE `routine_exercise`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=7;

--
-- AUTO_INCREMENT de la tabla `users`
--
ALTER TABLE `users`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3;

--
-- AUTO_INCREMENT de la tabla `user_activities`
--
ALTER TABLE `user_activities`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=7;

--
-- AUTO_INCREMENT de la tabla `user_type`
--
ALTER TABLE `user_type`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3;

--
-- Restricciones para tablas volcadas
--

--
-- Filtros para la tabla `exercises`
--
ALTER TABLE `exercises`
  ADD CONSTRAINT `fk_exercises_machines` FOREIGN KEY (`machine`) REFERENCES `machines` (`id`) ON DELETE CASCADE ON UPDATE CASCADE,
  ADD CONSTRAINT `fk_exercises_muscles` FOREIGN KEY (`muscle`) REFERENCES `muscles` (`id`) ON DELETE CASCADE ON UPDATE CASCADE;

--
-- Filtros para la tabla `routine_exercise`
--
ALTER TABLE `routine_exercise`
  ADD CONSTRAINT `fk_routine_exercise_exercise` FOREIGN KEY (`exercise`) REFERENCES `exercises` (`id`) ON DELETE CASCADE ON UPDATE CASCADE,
  ADD CONSTRAINT `fk_routine_exercise_routine` FOREIGN KEY (`routine`) REFERENCES `routine` (`id`) ON DELETE CASCADE ON UPDATE CASCADE;

--
-- Filtros para la tabla `users`
--
ALTER TABLE `users`
  ADD CONSTRAINT `fk_users_user_type` FOREIGN KEY (`user_type`) REFERENCES `user_type` (`id`) ON UPDATE CASCADE;

--
-- Filtros para la tabla `user_activities`
--
ALTER TABLE `user_activities`
  ADD CONSTRAINT `fk_user_activities_exercise` FOREIGN KEY (`exercise_id`) REFERENCES `exercises` (`id`) ON DELETE SET NULL ON UPDATE CASCADE,
  ADD CONSTRAINT `fk_user_activities_user` FOREIGN KEY (`user_id`) REFERENCES `users` (`id`) ON DELETE CASCADE ON UPDATE CASCADE;
COMMIT;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
