#!/usr/bin/env python3
"""
VLA Inference Module: Use Vision Language Models to score/select grasp candidates
基于OpenVLA的语言视觉模型实现gras候选评分与选择

Resources:
- GitHub: https://github.com/jpl-prpl/openvla
- Model Card: https://huggingface.co/openvla/openvla-7b-v1
"""

import rospy
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from io import BytesIO
import threading
import queue

# VLA imports (需要: pip install openvla transformers)
try:
    from transformers import AutoProcessor, AutoModelForVision2Seq
    VLA_AVAILABLE = True
except ImportError:
    VLA_AVAILABLE = False
    rospy.logwarn("OpenVLA not installed. Install with: pip install openvla transformers")


class VLAInferenceEngine:
    """
    Vision Language Model推理引擎
    用于评分/选择grasp候选
    """
    
    def __init__(self, model_checkpoint='openvla/openvla-7b-v1', use_lora=False, lora_path=None):
        """
        初始化VLA模型
        
        Args:
            model_checkpoint: HuggingFace模型checkpoint路径
            use_lora: 是否使用LoRA微调权重
            lora_path: LoRA权重文件路径
        """
        if not VLA_AVAILABLE:
            raise RuntimeError("OpenVLA not available. Install with: pip install openvla transformers")
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model_name = model_checkpoint
        self.use_lora = use_lora
        
        rospy.loginfo(f"Loading VLA model: {model_checkpoint} on {self.device}")
        
        try:
            # 加载processor和模型
            self.processor = AutoProcessor.from_pretrained(model_checkpoint, trust_remote_code=True)
            self.model = AutoModelForVision2Seq.from_pretrained(
                model_checkpoint,
                torch_dtype=torch.float16 if self.device == 'cuda' else torch.float32,
                device_map='auto',
                trust_remote_code=True
            )
            
            # 加载LoRA权重（如果提供）
            if use_lora and lora_path:
                rospy.loginfo(f"Loading LoRA weights from {lora_path}")
                # 使用peft库加载LoRA
                try:
                    from peft import PeftModel
                    self.model = PeftModel.from_pretrained(self.model, lora_path)
                except ImportError:
                    rospy.logwarn("peft not installed. Skipping LoRA loading.")
            
            self.model.eval()
            rospy.loginfo("VLA model loaded successfully")
        
        except Exception as e:
            rospy.logerr(f"Failed to load VLA model: {e}")
            raise
    
    @torch.no_grad()
    def score_candidates(self, image, candidates, task_description, temperature=0.7):
        """
        评分grasp候选
        
        Args:
            image: RGB图像 (H, W, 3) uint8 numpy array 或 PIL Image
            candidates: List[GraspCandidate] 候选对象列表
            task_description: str，任务描述（如"grasp the red cube"）
            temperature: float，采样温度（用于softmax）
        
        Returns:
            scores: np.array of shape (len(candidates),) in [0, 1]
            reasoning: list of str，模型对每个候选的解释（可选）
        """
        if len(candidates) == 0:
            return np.array([]), []
        
        # 转换image为PIL
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image)
        
        scores = []
        reasoning = []
        
        for i, candidate in enumerate(candidates):
            try:
                # 构造prompt
                prompt = self._build_prompt(candidate, task_description, i, len(candidates))
                
                # 前向推理
                inputs = self.processor(
                    images=image,
                    text=prompt,
                    return_tensors='pt'
                ).to(self.device)
                
                if self.device == 'cuda':
                    inputs = {k: v.half() if v.dtype == torch.float32 else v for k, v in inputs.items()}
                
                # 模型前向
                outputs = self.model(**inputs)
                
                # 提取logits并计算概率
                if hasattr(outputs, 'logits'):
                    logits = outputs.logits[:, -1, :]  # 最后一个token
                    # Softmax归一化
                    probs = F.softmax(logits / temperature, dim=-1)
                    score = probs.max().item()
                else:
                    # 备用方案：直接用loss作为相反分数
                    if hasattr(outputs, 'loss') and outputs.loss is not None:
                        score = 1.0 / (1.0 + outputs.loss.item())
                    else:
                        score = 0.5
                
                scores.append(score)
                reasoning.append(f"Candidate {i}: score={score:.3f}")
            
            except Exception as e:
                rospy.logwarn(f"Failed to score candidate {i}: {e}")
                scores.append(0.0)
                reasoning.append(f"Candidate {i}: failed - {str(e)[:50]}")
        
        return np.array(scores), reasoning
    
    def select_candidate(self, image, candidates, task_description, temperature=0.7, top_k=1):
        """
        选择最佳候选
        
        Args:
            image: 输入图像
            candidates: 候选列表
            task_description: 任务描述
            temperature: 采样温度
            top_k: 返回前k个候选
        
        Returns:
            selected_candidates: List[(candidate, score)]
            scores: np.array of scores
        """
        scores, reasoning = self.score_candidates(image, candidates, task_description, temperature)
        
        if len(scores) == 0:
            return [], scores
        
        # 获取top-k
        top_indices = np.argsort(scores)[-top_k:][::-1]
        selected = [(candidates[i], scores[i]) for i in top_indices]
        
        rospy.loginfo(f"VLA selection: {reasoning[top_indices[0]]}")
        return selected, scores
    
    def _build_prompt(self, candidate, task_description, candidate_idx, total_candidates):
        """
        构造VLA的prompt
        
        Args:
            candidate: GraspCandidate对象
            task_description: 任务描述
            candidate_idx: 候选索引
            total_candidates: 总候选数
        
        Returns:
            prompt: str
        """
        # 基础信息
        prompt_parts = [
            f"Task: {task_description}",
            f"Candidate {candidate_idx + 1}/{total_candidates}",
        ]
        
        # 候选的几何信息（如果GraspCandidate有这些属性）
        if hasattr(candidate, 'approach_vector') and candidate.approach_vector is not None:
            approach = candidate.approach_vector
            prompt_parts.append(f"Approach direction: ({approach[0]:.3f}, {approach[1]:.3f}, {approach[2]:.3f})")
        
        if hasattr(candidate, 'grasp_pose') and candidate.grasp_pose is not None:
            prompt_parts.append(f"Grasp pose available")
        
        if hasattr(candidate, 'feasibility_info') and candidate.feasibility_info:
            feasible = candidate.feasibility_info.get('ik_feasible', False)
            prompt_parts.append(f"IK feasible: {feasible}")
        
        # 优先级/评分信息
        if hasattr(candidate, 'priority') and candidate.priority is not None:
            prompt_parts.append(f"Prior priority: {candidate.priority}")
        
        prompt_parts.append("Is this a good grasp candidate? Respond yes or no.")
        
        return " ".join(prompt_parts)
    
    def batch_score_images(self, images, prompt, batch_size=4):
        """
        批量评分多张图像（针对视频或序列）
        
        Args:
            images: List[np.ndarray] 图像列表
            prompt: str 统一的prompt
            batch_size: 批大小
        
        Returns:
            scores: np.array of shape (len(images),)
        """
        scores = []
        
        for i in range(0, len(images), batch_size):
            batch = images[i:i+batch_size]
            batch_images = [Image.fromarray(img) if isinstance(img, np.ndarray) else img for img in batch]
            
            try:
                inputs = self.processor(
                    images=batch_images,
                    text=prompt,
                    return_tensors='pt',
                    padding=True
                ).to(self.device)
                
                if self.device == 'cuda':
                    inputs = {k: v.half() if v.dtype == torch.float32 else v for k, v in inputs.items()}
                
                with torch.no_grad():
                    outputs = self.model(**inputs)
                    logits = outputs.logits[:, -1, :]
                    probs = F.softmax(logits, dim=-1)
                    batch_scores = probs.max(dim=-1).values.cpu().numpy()
                
                scores.extend(batch_scores)
            
            except Exception as e:
                rospy.logerr(f"Batch inference failed: {e}")
                scores.extend([0.5] * len(batch))
        
        return np.array(scores)


class VLAAdapterNode:
    """
    ROS节点：适配VLA到grasp pipeline
    订阅感知输出，使用VLA评分候选，发布选择结果
    """
    
    def __init__(self):
        rospy.init_node('vla_adapter_node', log_level=rospy.INFO)
        
        # 参数
        model_ckpt = rospy.get_param('~model_checkpoint', 'openvla/openvla-7b-v1')
        use_lora = rospy.get_param('~use_lora', False)
        lora_path = rospy.get_param('~lora_path', '')
        temperature = rospy.get_param('~temperature', 0.7)
        
        # 初始化VLA
        try:
            self.vla = VLAInferenceEngine(model_ckpt, use_lora, lora_path if use_lora else None)
            self.temperature = temperature
        except Exception as e:
            rospy.logerr(f"Failed to initialize VLA: {e}")
            self.vla = None
        
        # 导入grasp_candidate_generator（需要在同一包中）
        try:
            from grasp_candidate_generator import GraspCandidateGenerator
            self.generator = GraspCandidateGenerator()
        except ImportError:
            rospy.logwarn("GraspCandidateGenerator not found. VLA adapter running in evaluation mode.")
            self.generator = None
        
        # Subscribers
        rospy.Subscriber('/target_cube_pose', ..., self.on_target_pose, queue_size=1)
        rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', ..., self.on_rgb, queue_size=1)
        
        # Publishers
        self.selected_pub = rospy.Publisher('/vla_selected_candidate', ..., queue_size=1)
        self.scores_pub = rospy.Publisher('/vla_candidate_scores', ..., queue_size=1)
        
        # 线程处理（异步推理，不阻塞ROS回调）
        self.inference_queue = queue.Queue(maxsize=1)
        self.inference_thread = threading.Thread(target=self._inference_worker, daemon=True)
        self.inference_thread.start()
        
        self.current_rgb = None
        self.current_target = None
        
        rospy.loginfo("VLA Adapter Node initialized")
    
    def on_rgb(self, msg):
        """Store latest RGB image"""
        try:
            from cv_bridge import CvBridge
            self.current_rgb = CvBridge().imgmsg_to_cv2(msg, 'BGR8')
        except Exception as e:
            rospy.logwarn(f"RGB conversion failed: {e}")
    
    def on_target_pose(self, msg):
        """
        当检测到目标时，生成候选并用VLA评分
        """
        if self.vla is None:
            rospy.logwarn("VLA not available")
            return
        
        if self.generator is None:
            rospy.logwarn("Generator not available")
            return
        
        try:
            # 生成候选
            candidates = self.generator.generate(msg)
            
            if self.current_rgb is None:
                rospy.logwarn("No RGB image available")
                return
            
            # 放入队列异步处理（避免阻塞）
            try:
                self.inference_queue.put_nowait((self.current_rgb.copy(), candidates, msg), timeout=0.1)
            except queue.Full:
                rospy.logdebug("Inference queue full, dropping frame")
        
        except Exception as e:
            rospy.logerr(f"Error in on_target_pose: {e}")
    
    def _inference_worker(self):
        """后台线程：处理VLA推理"""
        while not rospy.is_shutdown():
            try:
                rgb, candidates, target_msg = self.inference_queue.get(timeout=1)
                
                # VLA评分
                task_desc = "Grasp the cube and place it in the bin"
                selected, scores = self.vla.select_candidate(
                    rgb, candidates, task_desc, self.temperature, top_k=1)
                
                if selected:
                    best_candidate, best_score = selected[0]
                    rospy.loginfo(f"VLA selected candidate {best_candidate.id} with score {best_score:.3f}")
                    
                    # 发布结果（这里应该是自定义消息类型）
                    # self.selected_pub.publish(...)
                    
                    # 发布所有分数
                    # self.scores_pub.publish(...)
            
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Inference worker error: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = VLAAdapterNode()
    node.run()
