// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "main_window.h"

#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <iostream>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

// ROS2 includes for communication with the system
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std;

namespace {
using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

// Gauss Newton
g2o::OptimizationAlgorithm* createGauss() {
  auto linearSolverGN = std::make_unique<SlamLinearSolver>();
  linearSolverGN->setBlockOrdering(false);
  return new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<SlamBlockSolver>(std::move(linearSolverGN)));
}

// Levenberg
g2o::OptimizationAlgorithm* createLevenberg() {
  auto linearSolverLM = std::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  return new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));
}

}  // namespace

// ROS2 node for communication
class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("visualization_node") {
        // Create publishers for various topics
        exploration_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("exploration_command", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("calculated_path", 10);
    }

    void sendExplorationCommand(bool start) {
        auto msg = std_msgs::msg::Bool();
        msg.data = start;
        exploration_cmd_pub_->publish(msg);
    }

    void sendGoal(double x, double y) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.w = 1.0;
        goal_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

static shared_ptr<VisualizationNode> viz_node = nullptr;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUi(this);
    
    // Initialize ROS2 if not already initialized
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }
    
    // Create visualization node
    if (!viz_node) {
        viz_node = make_shared<VisualizationNode>();
        
        // Start ROS2 spinner in a separate thread
        std::thread([this]() {
            rclcpp::spin(viz_node);
        }).detach();
    }
    
    // Set initial status messages
    explorationStatus->setText("状态: 就绪");
    pathStatus->setText("路径状态: 无目标");
    mapInfo->setText("地图信息: 未初始化");
}

MainWindow::~MainWindow() {}

void MainWindow::on_actionLoad_triggered(bool) {
    viewer->graph->clear();
    QString filename = QFileDialog::getOpenFileName(
        this, "加载g2o文件", "", "g2o files (*.g2o);;所有文件 (*)");
    if (!filename.isNull()) {
        ifstream ifs(filename.toStdString().c_str());
        viewer->graph->load(ifs);
        string msg = "图已加载，顶点数量：" + to_string(viewer->graph->vertices().size()) +
                     "，边数量：" + to_string(viewer->graph->edges().size());
        cerr << msg << endl;
        mapInfo->setText(QString::fromStdString("地图信息: " + msg));
    }
    viewer->update();
    fixGraph();
}

void MainWindow::on_actionSave_triggered(bool) {
    QString filename = QFileDialog::getSaveFileName(this, "保存g2o文件", "",
                                                    "g2o files (*.g2o)");
    if (!filename.isNull()) {
        ofstream fout(filename.toStdString().c_str());
        viewer->graph->save(fout);
        if (fout.good()) {
            cerr << "已保存 " << filename.toStdString() << endl;
            QMessageBox::information(this, "成功", "地图已保存到: " + filename);
        } else {
            cerr << "保存文件时出错" << endl;
            QMessageBox::warning(this, "错误", "保存地图失败！");
        }
    }
}

void MainWindow::on_actionQuit_triggered(bool) { 
    rclcpp::shutdown();
    close(); 
}

void MainWindow::on_btnOptimize_clicked() {
    if (viewer->graph->vertices().size() == 0 ||
        viewer->graph->edges().size() == 0) {
        cerr << "图没有顶点/边" << endl;
        return;
    }

    viewer->graph->initializeOptimization();

    if (rbGauss->isChecked())
        viewer->graph->setAlgorithm(createGauss());
    else if (rbLevenberg->isChecked())
        viewer->graph->setAlgorithm(createLevenberg());
    else
        viewer->graph->setAlgorithm(createGauss());

    int maxIterations = spIterations->value();
    int iter = viewer->graph->optimize(maxIterations);
    if (maxIterations > 0 && !iter) {
        cerr << "优化失败，结果可能无效" << endl;
    }

    if (cbCovariances->isChecked()) {
        std::vector<std::pair<int, int> > cov_vertices;
        for (const auto& vertex_index : viewer->graph->vertices()) {
            auto* vertex =
                static_cast<g2o::OptimizableGraph::Vertex*>(vertex_index.second);
            if (!vertex->fixed())
                cov_vertices.emplace_back(vertex->hessianIndex(),
                                          vertex->hessianIndex());
        }
        viewer->covariances.clear(true);
        std::cerr << "计算协方差矩阵" << std::endl;
        bool cov_result =
            viewer->graph->computeMarginals(viewer->covariances, cov_vertices);
        viewer->drawCovariance = cov_result;
        std::cerr << (cov_result ? "完成。" : "失败") << std::endl;
    } else {
        viewer->drawCovariance = false;
    }
    viewer->update();
}

void MainWindow::on_actionShowRobotPath_toggled(bool checked) {
    // This would be implemented to toggle display of robot path
    cout << "机器人路径显示: " << (checked ? "开启" : "关闭") << endl;
}

void MainWindow::on_actionShowFrontiers_toggled(bool checked) {
    // This would be implemented to toggle display of frontier points
    cout << "边界点显示: " << (checked ? "开启" : "关闭") << endl;
}

void MainWindow::on_actionShowObstacles_toggled(bool checked) {
    // This would be implemented to toggle display of obstacles
    cout << "障碍物显示: " << (checked ? "开启" : "关闭") << endl;
}

void MainWindow::startExploration() {
    if (viz_node) {
        viz_node->sendExplorationCommand(true);
        explorationStatus->setText("状态: 探索中...");
        startExplorationButton->setEnabled(false);
        stopExplorationButton->setEnabled(true);
    }
}

void MainWindow::stopExploration() {
    if (viz_node) {
        viz_node->sendExplorationCommand(false);
        explorationStatus->setText("状态: 已停止");
        startExplorationButton->setEnabled(true);
        stopExplorationButton->setEnabled(false);
    }
}

void MainWindow::pauseExploration() {
    // Toggle pause/resume
    static bool paused = false;
    paused = !paused;
    pauseExplorationButton->setText(paused ? "继续" : "暂停");
    explorationStatus->setText(paused ? "状态: 已暂停" : "状态: 探索中...");
}

void MainWindow::calculatePath() {
    // In a real implementation, this would calculate a path to an interesting point
    // For now, we'll just update the status
    pathStatus->setText("路径状态: 已计算路径");
}

void MainWindow::navigateToGoal() {
    // In a real implementation, this would send a navigation goal
    // For now, we'll just update the status
    pathStatus->setText("路径状态: 导航中...");
}

void MainWindow::cancelGoal() {
    // In a real implementation, this would cancel the current navigation goal
    pathStatus->setText("路径状态: 无目标");
}

void MainWindow::saveMap() {
    QString filename = QFileDialog::getSaveFileName(this, "保存地图", "",
                                                    "地图文件 (*.pgm);;所有文件 (*)");
    if (!filename.isNull()) {
        // This would trigger the map saver node to save the occupancy grid
        QMessageBox::information(this, "保存地图", "地图保存功能将在实际系统中调用map_saver_node");
    }
}

void MainWindow::loadMap() {
    QString filename = QFileDialog::getOpenFileName(
        this, "加载地图", "", "地图文件 (*.pgm);;g2o文件 (*.g2o);;所有文件 (*)");
    if (!filename.isNull()) {
        // This would load a previously saved map
        mapInfo->setText("地图信息: 已加载 " + QFileInfo(filename).fileName());
        QMessageBox::information(this, "加载地图", "地图加载功能将在实际系统中加载预存地图");
    }
}

void MainWindow::resetMap() {
    viewer->graph->clear();
    viewer->update();
    mapInfo->setText("地图信息: 已重置");
    explorationStatus->setText("状态: 已重置");
    pathStatus->setText("路径状态: 无");
}

void MainWindow::fixGraph() {
    if (viewer->graph->vertices().size() == 0 ||
        viewer->graph->edges().size() == 0) {
        return;
    }

    // check for vertices to fix to remove DoF
    bool gaugeFreedom = viewer->graph->gaugeFreedom();
    g2o::OptimizableGraph::Vertex* gauge = viewer->graph->findGauge();
    if (gaugeFreedom) {
        if (!gauge) {
            cerr << "cannot find a vertex to fix in this thing" << endl;
            return;
        } else {
            cerr << "graph is fixed by node " << gauge->id() << endl;
            gauge->setFixed(true);
        }
    } else {
        cerr << "graph is fixed by priors" << endl;
    }

    viewer->graph->setVerbose(true);
    viewer->graph->computeActiveErrors();
}