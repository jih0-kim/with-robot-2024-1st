from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
import threading
from typing import TypeVar
import time

# stop_event = TypeVar("stop_event", bound=threading.Event)
stop_event = threading.Event()


class SearchEndException(Exception):
    pass


class Orient(Enum):
    X = "x"
    _X = "-x"
    Y = "y"
    _Y = "-y"


class DirType(Enum):
    LEFT = auto()
    RIGHT = auto()
    FORWARD = auto()
    BACKWARD = auto()


class State(Enum):
    ROTATE_READY = auto()
    ROTATE_START = auto()
    ROTATE_STOP = auto()
    RUN = auto()
    STOP = auto()


# Python에서는 프로토타입 기반의 객체 지향 프로그래밍을 직접적으로 지원하지 않지만,
# __getattr__ 메서드와 메타클래스를 활용하면 프로토타입 기반의 인터페이스를 구현할 수 있습니다. 다음과 같이 작성할 수 있습니다:
# class InterfaceMeta(type):
#     def __new__(cls, name, bases, attrs):
#         new_attrs = {}
#         for base in bases:
#             for attr_name, attr_value in base.__dict__.items():
#                 if not attr_name.startswith("_"):
#                     new_attrs[attr_name] = attr_value
#         attrs.update(new_attrs)
#         return super().__new__(cls, name, bases, attrs)

# class Interface(metaclass=InterfaceMeta):
# '''인터페이스정의 클래스'''


class MessageHandler:
    def print_log(self, message):
        """로깅을 한다"""

    def _send_message(self, **kwargs):
        """로봇에 메시지를 전달한다"""


class StateData:
    cur: any
    old: any

    def __init__(self, cur: any, old: any):
        self.cur = cur
        self.old = old

    def shift(self, new_data):
        self.old = self.cur
        self.cur = new_data

    def __eq__(self, __o: object) -> bool:
        return self.cur == __o

    def __repr__(self) -> str:
        return f"[cur:{self.cur}, old:{self.old}]"


@dataclass(slots=True, kw_only=True)
class Message:
    title: str = None
    data_type: str
    data: dict
    subject: str = None


class Chainable:
    @abstractmethod
    def check_condition(self):
        """"""


class Observer:
    # _msg_arrived: dict[str, bool] ={}
    # _msg: dict[str, Message]={}
    def __init__(self):
        self._msg_arrived: dict[str, bool] = {}
        self._msg: dict[str, Message] = {}
    
    def get_msg(self, subject: str = None, only_body: bool = False):
        while True:
            if not self._msg_arrived:
                time.sleep(0.05)
                continue
            # 단일처리인 경우 next로 첫번째 선택
            if not subject and self._msg:
                subject = next(iter(self._msg.keys()))
                break
            if self._msg_arrived.get(subject, False):
                break
            time.sleep(0.05)

        print_log(f"[Observer] get_msg: {subject}m/{self._msg}")
        # 플래그 초기화
        self._msg_arrived[subject] = False

        if only_body:
            return self._msg[subject].data

        return self._msg[subject]

    def update(self, message: Message):
        subject = message.subject
        self._msg[subject] = message
        self._msg_arrived[subject] = True


class Observable:
    _observe_map: dict[str, list[Observer]] = {}
    _subject: str = "node"

    # 비동기로 callback
    @classmethod
    def subscribe(cls, o: Observer, subject: str = None):
        """주제를 구독한다"""
        subject = subject or cls._subject
        if subject not in cls._observe_map:
            cls._observe_map[subject] = []
        cls._observe_map[subject].append(o)

    @classmethod
    def unsubscribe(cls, o: Observer, subject: str = None):
        """주제 구독을 해제한다"""
        subject = subject or cls._subject
        if subject in cls._observe_map:
            cls._observe_map[subject].remove(o)

    @classmethod
    def publish_(cls, data: object):
        cls.publish(Message(data_type=cls._subject, data=data, subject=cls._subject))

    @classmethod
    def publish(cls, message: Message, subject:str=None):
        """주제를 모든 구독자에게 전달한다."""
        subject = subject or cls._subject
        for o in cls._observe_map.get(subject, []):
            if callable(o):
                o(message)
            elif isinstance(o, Observer):
                o.update(message)


class EvHandle(threading.Thread):
    _lock = threading.Lock()
    action: tuple = (DirType.FORWARD, Orient.X)

    def __init__(self):
        super().__init__()

    def apply(self, **kwargs):
        stop_event.clear()
        self.setup(**kwargs)
        # thread 개시
        self.start()

    @abstractmethod
    def setup(self, **kwargs):
        """"""

    def _notifyall(self, subject: str, message: Message):
        # async def create_task():
        #     task = asyncio.create_task(
        #         Observable.publish(
        #             subject, message
        #         )
        #     )
        #     await asyncio.sleep(0)
        #     return task

        # asyncio.get_event_loop().run_until_complete(create_task())
        Observable.publish(message, subject)


def print_log(message: str, subject: str = "node"):
    Observable.publish(
        Message(subject=subject, data_type="log", data=message)
    )
