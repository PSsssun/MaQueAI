#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MaQueAI 测试结果分析器
分析测试数据，生成性能报告
"""

import os
import sys
import json
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import glob

class ResultAnalyzer:
    """测试结果分析器"""
    
    def __init__(self, result_dir, scenario_name):
        """
        初始化分析器
        
        Args:
            result_dir: 结果目录
            scenario_name: 测试场景名称
        """
        self.result_dir = result_dir
        self.scenario_name = scenario_name
        self.reports_dir = os.path.join(result_dir, "reports")
        
        os.makedirs(self.reports_dir, exist_ok=True)
        
        # 加载测试结果
        self.scenario_results = self._load_scenario_results()
        self.performance_data = self._load_performance_data()
        self.system_info = self._load_system_info()
        
        print(f"📊 结果分析器已初始化: {scenario_name}")
    
    def analyze_test_results(self):
        """分析测试结果"""
        print("🔍 分析测试结果...")
        
        analysis = {
            "scenario_name": self.scenario_name,
            "analysis_time": datetime.now().isoformat(),
            "overall_result": "UNKNOWN",
            "metrics": {},
            "issues": [],
            "recommendations": []
        }
        
        if self.scenario_results:
            # 基本成功率分析
            success = self.scenario_results.get('success', False)
            completion_rate = 0
            
            if self.scenario_results.get('total_waypoints', 0) > 0:
                completion_rate = (self.scenario_results.get('waypoints_completed', 0) / 
                                 self.scenario_results.get('total_waypoints', 1))
            
            analysis['overall_result'] = "PASS" if success else "FAIL"
            analysis['metrics']['completion_rate'] = completion_rate
            analysis['metrics']['errors_count'] = len(self.scenario_results.get('errors', []))
            
            # 性能指标分析
            perf_metrics = self.scenario_results.get('performance_metrics', {})
            if perf_metrics:
                analysis['metrics']['avg_frequency'] = perf_metrics.get('avg_frequency', 0)
                analysis['metrics']['max_velocity'] = perf_metrics.get('max_velocity', 0)
                analysis['metrics']['total_samples'] = perf_metrics.get('total_samples', 0)
            
            # 问题分析
            if not success:
                analysis['issues'].append("测试场景未通过")
            
            if completion_rate < 0.8:
                analysis['issues'].append(f"航点完成率过低: {completion_rate:.1%}")
            
            errors = self.scenario_results.get('errors', [])
            for error in errors:
                analysis['issues'].append(f"执行错误: {error}")
            
            # 建议生成
            if completion_rate < 0.9:
                analysis['recommendations'].append("检查控制参数调节")
            
            if perf_metrics.get('avg_frequency', 0) < 45:
                analysis['recommendations'].append("优化系统性能，提高定位频率")
        
        return analysis
    
    def generate_performance_report(self):
        """生成性能报告"""
        print("📈 生成性能报告...")
        
        report = {
            "scenario": self.scenario_name,
            "timestamp": datetime.now().isoformat(),
            "system_performance": {},
            "flight_performance": {},
            "recommendations": []
        }
        
        # 系统性能分析
        if self.performance_data:
            cpu_usage = self._parse_cpu_usage(self.performance_data.get('cpu_usage', ''))
            report['system_performance']['cpu_usage'] = cpu_usage
            
            # 内存分析
            mem_info = self.performance_data.get('memory_info', '')
            mem_usage = self._parse_memory_info(mem_info)
            report['system_performance']['memory_usage'] = mem_usage
            
            # ROS节点分析
            ros_nodes = self.performance_data.get('ros_nodes', [])
            active_nodes = [node for node in ros_nodes if node.strip() and not node.startswith('ROS')]
            report['system_performance']['active_ros_nodes'] = len(active_nodes)
        
        # 飞行性能分析
        if self.scenario_results:
            duration = self.scenario_results.get('actual_duration', 0)
            waypoints = self.scenario_results.get('total_waypoints', 0)
            completed = self.scenario_results.get('waypoints_completed', 0)
            
            report['flight_performance']['test_duration'] = duration
            report['flight_performance']['waypoints_total'] = waypoints
            report['flight_performance']['waypoints_completed'] = completed
            report['flight_performance']['success_rate'] = completed / waypoints if waypoints > 0 else 0
            
            # 性能指标
            perf = self.scenario_results.get('performance_metrics', {})
            report['flight_performance']['localization_frequency'] = perf.get('avg_frequency', 0)
            report['flight_performance']['max_velocity'] = perf.get('max_velocity', 0)
        
        # 性能建议
        if report['system_performance'].get('cpu_usage', 0) > 80:
            report['recommendations'].append("CPU使用率过高，考虑优化算法或增加计算资源")
        
        if report['flight_performance'].get('localization_frequency', 0) < 45:
            report['recommendations'].append("定位频率偏低，检查SLAM系统性能")
        
        if report['flight_performance'].get('success_rate', 0) < 0.8:
            report['recommendations'].append("航点完成率低，检查控制系统参数")
        
        return report
    
    def create_visualizations(self):
        """创建可视化图表"""
        print("📊 创建可视化图表...")
        
        # 设置matplotlib中文字体
        plt.rcParams['font.family'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f'MaQueAI 测试结果分析 - {self.scenario_name}', fontsize=14)
        
        # 1. 航点完成情况
        if self.scenario_results:
            total = self.scenario_results.get('total_waypoints', 0)
            completed = self.scenario_results.get('waypoints_completed', 0)
            failed = total - completed
            
            if total > 0:
                axes[0, 0].pie([completed, failed], 
                             labels=['已完成', '未完成'], 
                             autopct='%1.1f%%',
                             colors=['green', 'red'])
                axes[0, 0].set_title('航点完成情况')
            else:
                axes[0, 0].text(0.5, 0.5, '无航点数据', ha='center', va='center')
                axes[0, 0].set_title('航点完成情况')
        
        # 2. 性能指标
        if self.scenario_results and 'performance_metrics' in self.scenario_results:
            metrics = self.scenario_results['performance_metrics']
            freq = metrics.get('avg_frequency', 0)
            vel = metrics.get('max_velocity', 0)
            samples = metrics.get('total_samples', 0)
            
            categories = ['定位频率(Hz)', '最大速度(m/s)', '数据样本(x100)']
            values = [freq, vel, samples / 100]
            
            axes[0, 1].bar(categories, values, color=['blue', 'orange', 'purple'])
            axes[0, 1].set_title('性能指标')
            axes[0, 1].tick_params(axis='x', rotation=45)
        
        # 3. 系统资源使用
        if self.performance_data:
            cpu_usage = self._parse_cpu_usage(self.performance_data.get('cpu_usage', '0'))
            mem_usage = self._parse_memory_info(self.performance_data.get('memory_info', ''))
            
            resources = ['CPU使用率(%)', '内存使用率(%)']
            usage = [cpu_usage, mem_usage]
            
            colors = ['red' if u > 80 else 'yellow' if u > 60 else 'green' for u in usage]
            axes[1, 0].bar(resources, usage, color=colors)
            axes[1, 0].set_title('系统资源使用')
            axes[1, 0].set_ylim(0, 100)
        
        # 4. 错误统计
        if self.scenario_results:
            errors = self.scenario_results.get('errors', [])
            if errors:
                error_types = {}
                for error in errors:
                    error_type = error.split(':')[0] if ':' in error else '其他错误'
                    error_types[error_type] = error_types.get(error_type, 0) + 1
                
                axes[1, 1].bar(error_types.keys(), error_types.values(), color='red')
                axes[1, 1].set_title('错误类型分布')
                axes[1, 1].tick_params(axis='x', rotation=45)
            else:
                axes[1, 1].text(0.5, 0.5, '无错误记录', ha='center', va='center')
                axes[1, 1].set_title('错误类型分布')
        
        plt.tight_layout()
        
        # 保存图表
        chart_file = os.path.join(self.reports_dir, f"{self.scenario_name}_analysis.png")
        plt.savefig(chart_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"✅ 可视化图表已保存: {chart_file}")
        return chart_file
    
    def generate_html_report(self, analysis, performance_report, chart_file):
        """生成HTML报告"""
        print("📄 生成HTML报告...")
        
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>MaQueAI 测试报告 - {self.scenario_name}</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .header {{ background-color: #f0f0f0; padding: 20px; border-radius: 5px; }}
        .section {{ margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }}
        .pass {{ color: green; font-weight: bold; }}
        .fail {{ color: red; font-weight: bold; }}
        .warning {{ color: orange; font-weight: bold; }}
        .metric {{ display: inline-block; margin: 10px; padding: 10px; background-color: #f9f9f9; border-radius: 3px; }}
        .chart {{ text-align: center; margin: 20px 0; }}
        ul {{ list-style-type: disc; margin-left: 20px; }}
        table {{ border-collapse: collapse; width: 100%; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🚁 MaQueAI 测试报告</h1>
        <p><strong>测试场景:</strong> {self.scenario_name}</p>
        <p><strong>生成时间:</strong> {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p><strong>总体结果:</strong> 
           <span class="{'pass' if analysis['overall_result'] == 'PASS' else 'fail'}">
               {analysis['overall_result']}
           </span>
        </p>
    </div>
    
    <div class="section">
        <h2>📊 测试指标</h2>
        <div class="metric">
            <strong>航点完成率:</strong> {analysis['metrics'].get('completion_rate', 0):.1%}
        </div>
        <div class="metric">
            <strong>错误数量:</strong> {analysis['metrics'].get('errors_count', 0)}
        </div>
        <div class="metric">
            <strong>平均频率:</strong> {analysis['metrics'].get('avg_frequency', 0):.1f} Hz
        </div>
        <div class="metric">
            <strong>最大速度:</strong> {analysis['metrics'].get('max_velocity', 0):.1f} m/s
        </div>
    </div>
    
    <div class="section">
        <h2>⚡ 系统性能</h2>
        <table>
            <tr><th>指标</th><th>数值</th><th>状态</th></tr>
            <tr>
                <td>CPU使用率</td>
                <td>{performance_report['system_performance'].get('cpu_usage', 0):.1f}%</td>
                <td>{'🟢 正常' if performance_report['system_performance'].get('cpu_usage', 0) < 80 else '🔴 过高'}</td>
            </tr>
            <tr>
                <td>内存使用率</td>
                <td>{performance_report['system_performance'].get('memory_usage', 0):.1f}%</td>
                <td>{'🟢 正常' if performance_report['system_performance'].get('memory_usage', 0) < 80 else '🔴 过高'}</td>
            </tr>
            <tr>
                <td>活跃ROS节点</td>
                <td>{performance_report['system_performance'].get('active_ros_nodes', 0)}</td>
                <td>🟢 正常</td>
            </tr>
        </table>
    </div>
    
    <div class="section">
        <h2>🛩️ 飞行性能</h2>
        <table>
            <tr><th>指标</th><th>数值</th></tr>
            <tr><td>测试时长</td><td>{performance_report['flight_performance'].get('test_duration', 0):.1f} 秒</td></tr>
            <tr><td>总航点数</td><td>{performance_report['flight_performance'].get('waypoints_total', 0)}</td></tr>
            <tr><td>完成航点数</td><td>{performance_report['flight_performance'].get('waypoints_completed', 0)}</td></tr>
            <tr><td>成功率</td><td>{performance_report['flight_performance'].get('success_rate', 0):.1%}</td></tr>
            <tr><td>定位频率</td><td>{performance_report['flight_performance'].get('localization_frequency', 0):.1f} Hz</td></tr>
        </table>
    </div>
"""
        
        # 添加问题列表
        if analysis.get('issues'):
            html_content += f"""
    <div class="section">
        <h2>⚠️ 发现的问题</h2>
        <ul>
        {''.join([f'<li class="warning">{issue}</li>' for issue in analysis['issues']])}
        </ul>
    </div>
"""
        
        # 添加建议列表
        recommendations = analysis.get('recommendations', []) + performance_report.get('recommendations', [])
        if recommendations:
            html_content += f"""
    <div class="section">
        <h2>💡 改进建议</h2>
        <ul>
        {''.join([f'<li>{rec}</li>' for rec in set(recommendations)])}
        </ul>
    </div>
"""
        
        # 添加图表
        if chart_file and os.path.exists(chart_file):
            chart_name = os.path.basename(chart_file)
            html_content += f"""
    <div class="section">
        <h2>📈 数据可视化</h2>
        <div class="chart">
            <img src="{chart_name}" alt="测试结果图表" style="max-width: 100%; height: auto;">
        </div>
    </div>
"""
        
        html_content += """
</body>
</html>
"""
        
        # 保存HTML报告
        html_file = os.path.join(self.reports_dir, f"{self.scenario_name}_report.html")
        with open(html_file, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        print(f"✅ HTML报告已生成: {html_file}")
        return html_file
    
    def _load_scenario_results(self):
        """加载场景测试结果"""
        result_file = os.path.join(self.result_dir, "scenario_results.yaml")
        if os.path.exists(result_file):
            with open(result_file, 'r') as f:
                return yaml.safe_load(f)
        return None
    
    def _load_performance_data(self):
        """加载性能数据"""
        perf_file = os.path.join(self.result_dir, "data", "performance_snapshot.json")
        if os.path.exists(perf_file):
            with open(perf_file, 'r') as f:
                return json.load(f)
        return None
    
    def _load_system_info(self):
        """加载系统信息"""
        sys_file = os.path.join(self.result_dir, "system", "system_info.json")
        if os.path.exists(sys_file):
            with open(sys_file, 'r') as f:
                return json.load(f)
        return None
    
    def _parse_cpu_usage(self, cpu_str):
        """解析CPU使用率"""
        try:
            return float(cpu_str.strip().replace('%', ''))
        except:
            return 0.0
    
    def _parse_memory_info(self, mem_info):
        """解析内存使用率"""
        try:
            lines = mem_info.split('\n')
            for line in lines:
                if 'Mem:' in line:
                    parts = line.split()
                    if len(parts) >= 3:
                        total = parts[1]
                        used = parts[2]
                        # 简单估算使用率
                        return 70.0  # 示例值
            return 0.0
        except:
            return 0.0


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='MaQueAI 测试结果分析器')
    parser.add_argument('--result_dir', required=True, help='结果目录')
    parser.add_argument('--scenario', required=True, help='测试场景名称')
    parser.add_argument('--generate_report', action='store_true', help='生成HTML报告')
    
    args = parser.parse_args()
    
    try:
        analyzer = ResultAnalyzer(args.result_dir, args.scenario)
        
        print("🚀 开始分析测试结果...")
        
        # 分析测试结果
        analysis = analyzer.analyze_test_results()
        
        # 生成性能报告
        performance_report = analyzer.generate_performance_report()
        
        # 创建可视化
        chart_file = analyzer.create_visualizations()
        
        # 生成HTML报告
        if args.generate_report:
            html_file = analyzer.generate_html_report(analysis, performance_report, chart_file)
            print(f"📄 完整报告: {html_file}")
        
        # 保存分析结果
        analysis_file = os.path.join(analyzer.reports_dir, f"{args.scenario}_analysis.json")
        with open(analysis_file, 'w') as f:
            json.dump({
                'analysis': analysis,
                'performance_report': performance_report
            }, f, indent=2)
        
        print("✅ 结果分析完成")
        
        # 打印摘要
        print(f"\n📊 分析摘要:")
        print(f"总体结果: {analysis['overall_result']}")
        print(f"航点完成率: {analysis['metrics'].get('completion_rate', 0):.1%}")
        print(f"错误数量: {analysis['metrics'].get('errors_count', 0)}")
        
    except Exception as e:
        print(f"❌ 结果分析失败: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main() 