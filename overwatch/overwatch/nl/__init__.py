"""Natural-language targeting. Produces TargetQuery only; never touches flight."""

from .engine import CommandEngine, Reply
from .parser import Vocabulary, parse
from .schema import QueryError, TargetQuery, validate

__all__ = ["CommandEngine", "QueryError", "Reply", "TargetQuery", "Vocabulary", "parse", "validate"]
